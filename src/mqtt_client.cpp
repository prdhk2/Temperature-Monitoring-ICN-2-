#include "mqtt_client.h"

#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <cstdarg>
#include <cstring>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <time.h>

#include "config.h"
#include "debug_log.h"
#include "json_extract.h"
#include "mqtt_ca_cert.h"
#include "ota_manager.h"
#include "reset_reason.h"

// ResetReason mirrors esp_reset_reason_t by value (reset_reason.h).
static_assert(static_cast<int>(ESP_RST_POWERON) == ResetReason::POWERON, "reset code drift");
static_assert(static_cast<int>(ESP_RST_SW) == ResetReason::SW, "reset code drift");
static_assert(static_cast<int>(ESP_RST_PANIC) == ResetReason::PANIC, "reset code drift");
static_assert(static_cast<int>(ESP_RST_TASK_WDT) == ResetReason::TASK_WDT, "reset code drift");
static_assert(static_cast<int>(ESP_RST_BROWNOUT) == ResetReason::BROWNOUT, "reset code drift");

namespace {

constexpr size_t MAX_LINES_PER_BATCH = 8;  // 8 x 128 chars fits MQTT_PACKET_SIZE
constexpr uint16_t KEEPALIVE_S = 30;       // survives the pauses inside an OTA download
constexpr uint16_t SOCKET_TIMEOUT_S = 4;    // wait for CONNACK / packet bytes (busy-wait in PubSubClient)
constexpr size_t CMD_BUFFER_SIZE = 512;    // OTA command: ~150-char URL + 64-char hash + keys

char statusTopic[64];
char logTopic[64];
char cmdTopic[64];

volatile bool connected = false;
volatile uint32_t reconnects = 0;
bool confirmedHealthy = false;

// Used only from the MQTT task.
char payload[Config::MQTT_PACKET_SIZE];
PubSubClient *activeClient = nullptr;

const char *bootResetName() {
  static const char *name = ResetReason::name(static_cast<int>(esp_reset_reason()));
  return name;
}

// NTP (configTime() in setup()) sets the clock once Wi-Fi is up; before that
// time() is near 1970 and timestamps are sent as 0 ("unknown").
bool wallClockReady() { return time(nullptr) > 1600000000; }

unsigned long wallClockOrZero() {
  return wallClockReady() ? static_cast<unsigned long>(time(nullptr)) : 0UL;
}

const char *healthName(TcHealth h) {
  switch (h) {
    case TcHealth::OK: return "OK";
    case TcHealth::WAIT: return "WAIT";
    case TcHealth::FAULT: return "FAULT";
    case TcHealth::TIMEOUT: return "TIMEOUT";
    case TcHealth::OFFLINE: return "OFFLINE";
  }
  return "?";
}

// Bounded append into `payload`. Returns false once it would overflow.
bool append(size_t &used, const char *format, ...) __attribute__((format(printf, 2, 3)));
bool append(size_t &used, const char *format, ...) {
  if (used >= sizeof(payload)) return false;
  va_list args;
  va_start(args, format);
  int written = vsnprintf(payload + used, sizeof(payload) - used, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= sizeof(payload) - used) return false;
  used += static_cast<size_t>(written);
  return true;
}

// The retained status: what the fleet dashboard shows for this controller.
//
//   {"online":true,"uptime_s":..,"fw":"5.0.0","site":"depok_icn2","heap":..,
//    "minheap":..,"rssi":..,"ts":..,"reset":"POWERON",
//    "diag":{"kind":"depok_icn2","ota_trial":false,"part":"app0",
//      "tcs":[{"k":"Tmp_CmbJ1","v":812,"lim":900,"st":"OK","faults":0},...x5],
//      "alarm":false,"overheat":true,
//      "lfs":{"frames":120,"rejected":0,"age_s":1},
//      "upload":{"ok":42,"failed":0,"last_http":200,"last_ok_ts":..},
//      "wifi_reconnects":0,"mqtt_reconnects":1,"log_dropped":0}}
bool buildStatus() {
  const Icn2Status s = icn2Status();
  size_t used = 0;
  bool ok = append(used,
                   "{\"online\":true,\"uptime_s\":%lu,\"fw\":\"%s\",\"site\":\"%s\","
                   "\"heap\":%u,\"minheap\":%u,\"rssi\":%d,\"ts\":%lu,\"reset\":\"%s\","
                   "\"diag\":{\"kind\":\"depok_icn2\",\"ota_trial\":%s,\"part\":\"%s\",\"tcs\":[",
                   static_cast<unsigned long>(millis() / 1000UL), Config::FIRMWARE_VERSION,
                   Config::SITE, static_cast<unsigned>(ESP.getFreeHeap()),
                   static_cast<unsigned>(ESP.getMinFreeHeap()), static_cast<int>(WiFi.RSSI()),
                   wallClockOrZero(), bootResetName(), OtaManager::onTrial() ? "true" : "false",
                   OtaManager::runningPartition());
  for (int i = 0; i < ICN2_CHANNELS && ok; ++i) {
    ok = append(used, "%s{\"k\":\"%s\",\"v\":%d,\"lim\":%d,\"st\":\"%s\",\"faults\":%lu}",
                i ? "," : "", s.key[i], s.temp[i], s.limit[i], healthName(s.health[i]),
                static_cast<unsigned long>(s.faults[i]));
  }
  ok = ok && append(used,
                    "],\"alarm\":%s,\"overheat\":%s,"
                    "\"lfs\":{\"frames\":%lu,\"rejected\":%lu,\"age_s\":%ld},"
                    "\"upload\":{\"ok\":%lu,\"failed\":%lu,\"last_http\":%d,\"last_ok_ts\":%lu},"
                    "\"wifi_reconnects\":%lu,\"mqtt_reconnects\":%lu,\"log_dropped\":%lu}}",
                    s.alarm ? "true" : "false", s.overheat ? "true" : "false",
                    static_cast<unsigned long>(s.lfsFrames), static_cast<unsigned long>(s.lfsRejected),
                    s.lfsAgeS, static_cast<unsigned long>(s.uploadsOk),
                    static_cast<unsigned long>(s.uploadsFailed), s.lastHttp, s.lastUploadOkEpoch,
                    static_cast<unsigned long>(s.wifiReconnects),
                    static_cast<unsigned long>(reconnects),
                    static_cast<unsigned long>(logLinesDropped()));
  return ok;
}

void publishStatus(PubSubClient &client) {
  if (!buildStatus()) {
    // Never publish truncated JSON. Fall back to the minimal shape so the
    // node still shows online and the boot confirmation below still runs.
    logf("[MQTT] status payload overflow, sending minimal status");
    snprintf(payload, sizeof(payload),
             "{\"online\":true,\"uptime_s\":%lu,\"fw\":\"%s\",\"site\":\"%s\",\"heap\":%u,"
             "\"rssi\":%d,\"ts\":%lu,\"reset\":\"%s\"}",
             static_cast<unsigned long>(millis() / 1000UL), Config::FIRMWARE_VERSION, Config::SITE,
             static_cast<unsigned>(ESP.getFreeHeap()), static_cast<int>(WiFi.RSSI()),
             wallClockOrZero(), bootResetName());
  }
  if (!client.publish(statusTopic, payload, true)) return;

  // An image is healthy once it has stayed up a minute AND reached the
  // server; only then does it end an OTA trial (see ota_rollback_logic.h).
  if (!confirmedHealthy && millis() >= Config::BOOT_HEALTHY_AFTER_MS) {
    OtaManager::confirmBootHealthy();
    confirmedHealthy = true;
  }
}

void drainAndPublishLogs(PubSubClient &client) {
  static char lines[MAX_LINES_PER_BATCH][LOG_LINE_CAPACITY + 1];
  size_t count = drainLogLines(lines, MAX_LINES_PER_BATCH);
  if (count == 0) return;

  size_t used = 0;
  bool ok = append(used, "{\"lines\":[");
  for (size_t i = 0; i < count && ok; ++i) {
    // Lines were sanitised by logf(): no quotes, backslashes or control chars.
    ok = append(used, "%s\"%s\"", i ? "," : "", lines[i]);
  }
  ok = ok && append(used, "],\"ts\":%lu}", wallClockOrZero());
  if (ok) client.publish(logTopic, payload);
}

// OTA progress goes straight to the log topic - the log queue cannot be
// drained while this task is busy downloading - as the one fixed-format line
// the server parses (DeviceTelemetryIngestor::OTA_PROGRESS_PATTERN):
//   [OTA] progress stage=downloading pct=42 detail=
void publishOtaProgress(const char *stage, int pct, const char *detail) {
  char line[LOG_LINE_CAPACITY + 1];
  snprintf(line, sizeof(line), "[OTA] progress stage=%s pct=%d detail=%s", stage, pct,
           detail ? detail : "");
  for (char *c = line; *c; ++c) {
    if (*c == '"' || *c == '\\') *c = '\'';
  }
  serialDebugf("%s", line);
  if (!activeClient) return;

  char message[LOG_LINE_CAPACITY + 64];
  snprintf(message, sizeof(message), "{\"lines\":[\"%s\"],\"ts\":%lu}", line, wallClockOrZero());
  activeClient->publish(logTopic, message);
}

void handleOta(const char *json) {
  OtaManager::Command command = {};
  char site[24] = {0};
  bool fields = JsonExtract::string(json, "url", command.url, sizeof(command.url)) &&
                JsonExtract::string(json, "sha256", command.sha256, sizeof(command.sha256)) &&
                JsonExtract::string(json, "version", command.version, sizeof(command.version));
  if (!fields) {
    logf("[MQTT] OTA command missing url/sha256/version");
    publishOtaProgress("failed", -1, "OTA command missing url/sha256/version");
    return;
  }
  // The image must be built for this hardware: an OTA meant for another
  // site (e.g. the ICN-1 controller) would drive these pins wrongly.
  if (!JsonExtract::string(json, "site", site, sizeof(site)) || strcmp(site, Config::SITE) != 0) {
    logf("[OTA] rejected: release is for site '%s', this controller is '%s'", site, Config::SITE);
    publishOtaProgress("failed", -1, "release is for a different site");
    return;
  }
  // Blocks this task for up to ~90 s by design; loop() keeps running on the
  // other core. Returns only on failure.
  OtaManager::applyUpdate(command, publishOtaProgress);
}

void handleCommand(const char *json, PubSubClient &client) {
  char cmd[16] = {0};
  if (!JsonExtract::string(json, "cmd", cmd, sizeof(cmd))) {
    logf("[MQTT] cmd payload missing \"cmd\" field");
    return;
  }

  if (strcmp(cmd, "PING") == 0 || strcmp(cmd, "STATUS") == 0) {
    logf("[MQTT] %s received", cmd);
    publishStatus(client);
  } else if (strcmp(cmd, "RESTART") == 0) {
    logf("[MQTT] RESTART requested, rebooting");
    drainAndPublishLogs(client);
    client.loop();
    delay(300);
    ESP.restart();
  } else if (strcmp(cmd, "OTA") == 0) {
    handleOta(json);
  } else {
    logf("[MQTT] unrecognised cmd: %s", cmd);
  }
}

void mqttTask(void *) {
  WiFiClientSecure secureClient;
  secureClient.setCACert(ISRG_ROOT_X1_PEM);

  PubSubClient client(secureClient);
  client.setServer(Config::MQTT_BROKER_HOST, Config::MQTT_BROKER_PORT);
  client.setBufferSize(Config::MQTT_PACKET_SIZE);
  client.setKeepAlive(KEEPALIVE_S);
  // [FIX] PubSubClient::connect() menunggu CONNACK dengan busy-loop TANPA
  // yield selama socketTimeout (default 15 s). Kalau broker lambat/tidak
  // menjawab, IDLE0 kelaparan > 5 s -> task watchdog -> reboot berulang.
  // Timeout dipendekkan, dan task berjalan di prioritas idle (lihat beginMqtt).
  client.setSocketTimeout(SOCKET_TIMEOUT_S);
  activeClient = &client;

  client.setCallback([](char *topic, uint8_t *data, unsigned int length) {
    static char buffer[CMD_BUFFER_SIZE];
    if (strcmp(topic, cmdTopic) != 0) return;
    if (length >= sizeof(buffer)) {
      logf("[MQTT] payload too large (%u bytes), ignored", length);
      return;
    }
    memcpy(buffer, data, length);
    buffer[length] = '\0';
    handleCommand(buffer, *activeClient);
  });

  uint32_t lastConnectAttempt = 0;
  uint32_t lastStatusPublish = 0;
  uint32_t lastLogDrain = 0;
  bool firstAttempt = true;
  bool bootAnnounced = false;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      connected = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (!client.connected()) {
      connected = false;
      uint32_t now = millis();
      if (firstAttempt || now - lastConnectAttempt >= Config::MQTT_RECONNECT_MS) {
        firstAttempt = false;
        lastConnectAttempt = now;
        bool ok = client.connect(Config::DEVICE_ID, Config::MQTT_USERNAME, Config::MQTT_PASSWORD,
                                 statusTopic, 1, true, "{\"online\":false}");
        if (ok) {
          ++reconnects;
          client.subscribe(cmdTopic, 1);
          connected = true;
          logf("[MQTT] connected to %s", Config::MQTT_BROKER_HOST);
          if (!bootAnnounced) {
            bootAnnounced = true;
            logf("[BOOT] fw %s reset=%s part=%s", Config::FIRMWARE_VERSION, bootResetName(),
                 OtaManager::runningPartition());
            char note[LOG_LINE_CAPACITY + 1];
            if (OtaManager::takeRollbackNote(note, sizeof(note))) logf("%s", note);
          }
          publishStatus(client);
          lastStatusPublish = millis();
        } else {
          logf("[MQTT] connect failed, rc=%d, retry in %lus", client.state(),
               static_cast<unsigned long>(Config::MQTT_RECONNECT_MS / 1000UL));
        }
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    connected = true;
    client.loop();

    uint32_t now = millis();
    if (now - lastStatusPublish >= Config::MQTT_STATUS_INTERVAL_MS) {
      publishStatus(client);
      lastStatusPublish = now;
    }
    if (now - lastLogDrain >= Config::MQTT_LOG_DRAIN_INTERVAL_MS) {
      drainAndPublishLogs(client);
      lastLogDrain = now;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

}  // namespace

void beginMqtt() {
  snprintf(statusTopic, sizeof(statusTopic), "%s/%s/status", Config::SITE, Config::DEVICE_ID);
  snprintf(logTopic, sizeof(logTopic), "%s/%s/log", Config::SITE, Config::DEVICE_ID);
  snprintf(cmdTopic, sizeof(cmdTopic), "%s/%s/cmd", Config::SITE, Config::DEVICE_ID);

  if (!Config::mqttConfigured()) {
    logf("[MQTT] broker credentials not set (secrets.ini), remote status/OTA disabled");
    return;
  }
  // TLS + PubSubClient + the OTA download (HTTPClient over TLS) on one task.
  // [FIX] Prioritas = idle (0): busy-wait di dalam PubSubClient/WiFiClientSecure
  // tidak lagi bisa membuat IDLE0 kelaparan (FreeRTOS time-slicing antar task
  // berprioritas sama), jadi task watchdog tidak me-reset board. Task Wi-Fi/
  // lwIP sistem tetap di atasnya, dan loop() ada di core 1.
  BaseType_t created =
      xTaskCreatePinnedToCore(mqttTask, "mqtt", 12288, nullptr, tskIDLE_PRIORITY, nullptr, 0);
  if (created != pdPASS) logf("[MQTT] could not start the MQTT task (out of memory)");
}

bool mqttConnected() { return connected; }
uint32_t mqttReconnects() { return reconnects; }
