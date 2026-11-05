// ============================================================
//  ICN-2 Temperature Monitor - ESP32 firmware (PlatformIO)
//  5 thermocouple MAX31856 (TC1-TC5), HMI Nextion, panel relay/reject,
//  LFS flow meter, upload ke monitoring.xampah.com, fleet MQTT + OTA.
//
//  Dari sketch biodigester_esp32_v4.4.ino. PIN, protokol serial (HMI, panel,
//  LFS) dan layout EEPROM TIDAK berubah. Perbaikan bug ditandai [FIX].
//  Jaringan/fleet ada di include/config.h dan src/mqtt_client.cpp.
// ============================================================

#include <Arduino.h>
#include <Adafruit_MAX31856.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <climits>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <time.h>

#include "config.h"
#include "debug_log.h"
#include "mqtt_ca_cert.h"
#include "mqtt_client.h"
#include "ota_manager.h"
#include "sensor_filter.h"
#include "serial_frames.h"

// ============================================================
//  KONFIGURASI SENSOR - 5 TC (tidak berubah)
// ============================================================
#define NUM_TC 5
static_assert(NUM_TC == ICN2_CHANNELS, "status payload channel count");
static_assert(NUM_TC == SerialFrames::HmiParser::LIMIT_CHANNELS, "HMI limit frame channel count");

const int TC_CS_PINS[NUM_TC] = {
    15,  // TC1 -> Tmp_CmbJ1
    2,   // TC2 -> Tmp_PrlJ1_1
    4,   // TC3 -> Tmp_PrlJ1_2
    5,   // TC4 -> Tmp_PrlJ1_3
    13   // TC5 -> Tmp_PrlJ1_4
};

const char *const TC_JSON_KEY[NUM_TC] = {
    "Tmp_CmbJ1", "Tmp_PrlJ1_1", "Tmp_PrlJ1_2", "Tmp_PrlJ1_3", "Tmp_PrlJ1_4",
};

#define SENSOR_FAULT_VALUE 0

// ============================================================
//  KONFIGURASI PIN 
// ============================================================
#define pinMCU_SrlRX1 32
#define pinMCU_SrlTX1 33
#define pinMCU_SrlRX2 16
#define pinMCU_SrlTX2 17
// GPIO tidak dipakai firmware (didokumentasikan saja):
// GP_IO1 = 26, GP_IO2 = 25, GP_I_1 = 35, GP_I_2 = 34, GP_I_3 = 39, GP_I_4 = 36

// ============================================================
//  KONFIGURASI TIMING
// ============================================================
#define TC_CONVERSION_TIME_MS 160UL
#define TC_CONVERSION_TIMEOUT_MS 1000UL 
#define HMI_DATETIME_PERIOD_MS 500UL
#define HMI_REFRESH_MS 5000UL
#define LFS_POLL_PERIOD_MS 1000UL
#define SERVER_POST_PERIOD_MS 60000UL
#define WIFI_RETRY_PERIOD_MS 10000UL
#define HTTP_TIMEOUT_MS 8000
#define OUTPUT_SYNC_AFTER_MS 5000UL

// ============================================================
//  KONFIGURASI EEPROM (layout sama dengan v4.4: 5 x int32 + CRC32)
// ============================================================
#define EEPROM_SIZE 24
#define EEPROM_DATA_LEN 20
#define EEPROM_CRC_ADDR 20
#define EEPROM_DEFAULT_LIMIT 60
static_assert(EEPROM_DATA_LEN == NUM_TC * 4, "EEPROM layout");

// ============================================================
//  NTP
// ============================================================
const char *conChr_SrvNtp = "pool.ntp.org";
const long conLng_GMTOfs = 7 * 3600;
const int conInt_DayLghOfs = 0;

// ============================================================
//  KONSTANTA PANEL / HMI
// ============================================================
const int conInt_PnlMan = 0;
const int conInt_PnlLFS = 1;
const int conInt_PnlStg = 2;

const int TC1_OVERHEAT_LIMIT = 100;  // varInt_TmpTC1OvH di v4.4
const uint16_t HMI_COLOR_ALARM = 63488;
const uint16_t HMI_COLOR_NORMAL = 2024;

const char *const TC_HMI_LABEL[NUM_TC] = {
    "PANEL_MAIN.N_TmpTC1", "PANEL_MAIN.N_TmpTC2", "PANEL_MAIN.N_TmpTC3",
    "PANEL_MAIN.N_TmpTC4", "PANEL_MAIN.N_TmpTC5",
};
const char *const TC_STG_H_LABEL[NUM_TC] = {
    "PANEL_STNG.H_TmpTC1Lmt", "PANEL_STNG.H_TmpTC2Lmt", "PANEL_STNG.H_TmpTC3Lmt",
    "PANEL_STNG.H_TmpTC4Lmt", "PANEL_STNG.H_TmpTC5Lmt",
};
const char *const TC_STG_N_LABEL[NUM_TC] = {
    "PANEL_STNG.N_TmpTC1Lmt", "PANEL_STNG.N_TmpTC2Lmt", "PANEL_STNG.N_TmpTC3Lmt",
    "PANEL_STNG.N_TmpTC4Lmt", "PANEL_STNG.N_TmpTC5Lmt",
};

// ============================================================
//  SENSOR
// ============================================================
enum SensorState { TC_IDLE, TC_TRIGGERED, TC_READY_READ };

struct SensorTC {
  Adafruit_MAX31856 *driver = nullptr;
  SensorFilter filter;
  SensorState state = TC_IDLE;
  unsigned long triggerTime = 0;
  int rawValue = 0;
  int filteredValue = 0;
  int limitValue = EEPROM_DEFAULT_LIMIT;
  bool limitFlag = false;
  bool isOnline = false;
  bool isFault = false;
  bool timedOut = false;
  bool hasReading = false;
  uint8_t lastFaultBits = 0;
  uint32_t faultCount = 0;
  // Apa yang terakhir dikirim ke HMI (untuk tidak membanjiri UART 9600).
  int hmiVal = INT_MIN;
  uint16_t hmiColor = 0;
  bool hmiColorSent = false;
  unsigned long hmiSentMs = 0;
};

SensorTC sensors[NUM_TC];

HardwareSerial hrdWre_SrlP1_HMI(1);
HardwareSerial hrdWre_SrlP2_Rjc(2);

// Serial (UART0) = bus LFS flow meter. Hanya protokol LFS yang ditulis ke
// sini di build release (lihat Config::SERIAL_DEBUG).
SerialFrames::LineAssembler lfsLine;
SerialFrames::HmiParser hmiParser;
SerialFrames::LineAssembler rjcLine;

// ============================================================
//  STATE GLOBAL
// ============================================================
bool varBol_TmpTCn = false;     // paksa kirim ulang status alarm ke panel
bool varBol_TmpTCnLmt = false;  // status alarm limit yang terakhir dikirim
bool varBol_TmpTC1OvH = false;  // status over-heat TC1 yang terakhir dikirim
bool varBol_OutputsSynced = false;

bool varBol_WFiCnt = false;
volatile uint32_t varU32_WifiReconnects = 0;
unsigned long varLng_WFiTmeStrChkCnt = 0;
unsigned long varLng_DMDTmeStrTmpVal = 0;
unsigned long varLng_LFSTmeStrGetDta = 0;
unsigned long varLng_DbsTmeStrTmpVal = 0;

int varInt_PgeNmb = conInt_PnlMan;
bool varBol_RL[8] = {false, false, false, false, false, false, false, false};

volatile uint32_t varU32_LfsFrames = 0;
volatile uint32_t varU32_LfsRejected = 0;
volatile unsigned long varLng_LfsLastOkMs = 0;
volatile bool varBol_LfsSeen = false;

// ============================================================
//  UPLOAD HTTP - task FreeRTOS di core 0
// ============================================================
struct HttpPayload {
  char json[256];
};

QueueHandle_t httpQueue = nullptr;
volatile uint32_t varU32_UploadOk = 0;
volatile uint32_t varU32_UploadFailed = 0;
volatile int varInt_UploadLastHttp = 0;
volatile unsigned long varLng_UploadLastOkEpoch = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void HMISfx();
void rjcSend(const char *code);
void hmiShowSensor(int i, unsigned long now, bool force);
void _HMISendLimit(int i);
void EEPROMWR_();
bool EEPROMRD_();
void WiFi_Config();
void Wifi_Handle(unsigned long now);
void Wifi_SendData();
void Task_SensorRead(unsigned long now);
void Task_HMIDatetime(unsigned long now);
void Task_LFSPoll(unsigned long now);
void Task_SerialRx(unsigned long now);
void ProcessHMICommand();
void ProcessRjcCommand(const char *cmd);
void ProcessLFSData(const char *line, size_t length, unsigned long now);
void updateAlarmOutputs(unsigned long now);
void RjcRelay(int idx, bool on);
void Bol_TmpTCnTru();

// ============================================================
//  HELPER SERIAL HMI / PANEL
// ============================================================
void HMISfx() {
  hrdWre_SrlP1_HMI.write(0xff);
  hrdWre_SrlP1_HMI.write(0xff);
  hrdWre_SrlP1_HMI.write(0xff);
}

void rjcSend(const char *code) {
  hrdWre_SrlP2_Rjc.print(code);
  hrdWre_SrlP2_Rjc.write(0x0d);
}

// [FIX] v4.4 menulis nilai + warna setiap ~160 ms per sensor (5 sensor x
// ~30 byte x 6/s mendekati kapasitas 9600 baud), sehingga loop() tertahan
// menunggu UART. Sekarang hanya saat berubah, plus refresh tiap 5 s supaya
// HMI yang restart tetap terisi. [FIX] Warna kembali hijau setelah fault
// hilang (v4.4: tetap merah sampai status limit berubah).
void hmiShowSensor(int i, unsigned long now, bool force) {
  SensorTC &s = sensors[i];
  bool bad = !s.isOnline || s.isFault;
  int val = bad ? SENSOR_FAULT_VALUE : s.filteredValue;
  uint16_t color = (bad || s.limitFlag) ? HMI_COLOR_ALARM : HMI_COLOR_NORMAL;
  bool refresh = force || (now - s.hmiSentMs >= HMI_REFRESH_MS);

  if (refresh || !s.hmiColorSent || color != s.hmiColor) {
    hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
    hrdWre_SrlP1_HMI.print(".pco=");
    hrdWre_SrlP1_HMI.print(color);
    HMISfx();
    s.hmiColor = color;
    s.hmiColorSent = true;
  }
  if (refresh || val != s.hmiVal) {
    hrdWre_SrlP1_HMI.print(TC_HMI_LABEL[i]);
    hrdWre_SrlP1_HMI.print(".val=");
    hrdWre_SrlP1_HMI.print(val);
    HMISfx();
    s.hmiVal = val;
  }
  if (refresh) s.hmiSentMs = now;
}

void _HMISendLimit(int i) {
  hrdWre_SrlP1_HMI.print(TC_STG_H_LABEL[i]);
  hrdWre_SrlP1_HMI.print(".val=");
  hrdWre_SrlP1_HMI.print(sensors[i].limitValue);
  HMISfx();
  hrdWre_SrlP1_HMI.print(TC_STG_N_LABEL[i]);
  hrdWre_SrlP1_HMI.print(".val=");
  hrdWre_SrlP1_HMI.print(sensors[i].limitValue);
  HMISfx();
}

// Paksa loop() mengirim ulang status alarm ke panel.
void Bol_TmpTCnTru() { varBol_TmpTCn = true; }

// ============================================================
//  STATUS UNTUK FLEET (dibaca task MQTT, core 0)
// ============================================================
Icn2Status icn2Status() {
  Icn2Status st = {};
  unsigned long now = millis();
  for (int i = 0; i < NUM_TC; ++i) {
    const SensorTC &s = sensors[i];
    st.key[i] = TC_JSON_KEY[i];
    st.limit[i] = s.limitValue;
    st.faults[i] = s.faultCount;
    if (!s.isOnline) {
      st.health[i] = TcHealth::OFFLINE;
    } else if (s.isFault) {
      st.health[i] = s.timedOut ? TcHealth::TIMEOUT : TcHealth::FAULT;
    } else if (!s.hasReading) {
      st.health[i] = TcHealth::WAIT;
    } else {
      st.health[i] = TcHealth::OK;
    }
    st.temp[i] = st.health[i] == TcHealth::OK ? s.filteredValue : SENSOR_FAULT_VALUE;
  }
  st.alarm = varBol_TmpTCnLmt;
  st.overheat = varBol_TmpTC1OvH;
  st.lfsFrames = varU32_LfsFrames;
  st.lfsRejected = varU32_LfsRejected;
  st.lfsAgeS = varBol_LfsSeen ? static_cast<long>((now - varLng_LfsLastOkMs) / 1000UL) : -1;
  st.uploadsOk = varU32_UploadOk;
  st.uploadsFailed = varU32_UploadFailed;
  st.lastHttp = varInt_UploadLastHttp;
  st.lastUploadOkEpoch = varLng_UploadLastOkEpoch;
  st.wifiReconnects = varU32_WifiReconnects;
  return st;
}

// ============================================================
//  UPLOAD HTTP
// ============================================================
// Satu POST. Mengembalikan kode HTTP (> 0) atau error transport (< 0).
int postOnce(const char *json) {
  WiFiClientSecure secureClient;
  WiFiClient plainClient;
  HTTPClient http;

  // [FIX] v4.4 memakai setInsecure() (sertifikat tidak dicek). Sekarang
  // https:// diverifikasi dengan root Let's Encrypt; http:// tetap bisa
  // untuk server uji di meja.
  bool https = strncmp(Config::SERVER_URL, "https://", 8) == 0;
  bool begun;
  if (https) {
    secureClient.setCACert(ISRG_ROOT_X1_PEM);
    begun = http.begin(secureClient, Config::SERVER_URL);
  } else {
    begun = http.begin(plainClient, Config::SERVER_URL);
  }
  if (!begun) return HTTPC_ERROR_CONNECTION_REFUSED;

  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  if (Config::apiKeyConfigured()) http.addHeader("X-API-KEY", Config::API_KEY);

  int code = http.POST(reinterpret_cast<uint8_t *>(const_cast<char *>(json)), strlen(json));
  if (code >= 300) {
    String body = http.getString();
    logf("[HTTP] tempMonitor2 http=%d %.70s", code, body.c_str());
  }
  http.end();
  return code;
}

void Task_HttpPost(void *) {
  HttpPayload payload;
  for (;;) {
    if (xQueueReceive(httpQueue, &payload, portMAX_DELAY) != pdTRUE) continue;

    int code = 0;
    for (int attempt = 1; attempt <= 3; ++attempt) {
      if (WiFi.status() != WL_CONNECTED) {
        code = HTTPC_ERROR_NOT_CONNECTED;
        logf("[HTTP] skip upload, WiFi not connected");
        break;
      }
      if (OtaManager::inProgress()) {
        code = HTTPC_ERROR_NOT_CONNECTED;
        logf("[HTTP] skip upload, OTA download running");
        break;
      }
      code = postOnce(payload.json);
      if (code >= 200 && code < 300) break;
      logf("[HTTP] upload attempt %d failed, http=%d", attempt, code);
      // [FIX] v4.4 menganggap semua kode > 0 sukses (401/422/500 juga), dan
      // memanggil http.end() dua kali. 4xx tidak di-retry (tidak akan berubah).
      if (code >= 400 && code < 500) break;
      if (attempt < 3) vTaskDelay(pdMS_TO_TICKS(2000));
    }

    varInt_UploadLastHttp = code;
    if (code >= 200 && code < 300) {
      ++varU32_UploadOk;
      time_t nowEpoch = time(nullptr);
      if (nowEpoch > 1600000000) varLng_UploadLastOkEpoch = static_cast<unsigned long>(nowEpoch);
    } else {
      ++varU32_UploadFailed;
      if (code == 401) logf("[HTTP] API key rejected (401) - rebuild with a valid DP_API_KEY");
    }
  }
}

void Wifi_SendData() {
  if (WiFi.status() != WL_CONNECTED) {
    logf("[HTTP] skip upload, WiFi not connected");
    return;
  }
  if (!httpQueue) return;

  // Sensor offline / fault / belum ada bacaan dikirim 0 (sama dengan v4.4).
  int v[NUM_TC];
  for (int i = 0; i < NUM_TC; ++i) {
    const SensorTC &s = sensors[i];
    v[i] = (s.isOnline && !s.isFault && s.hasReading) ? s.filteredValue : SENSOR_FAULT_VALUE;
  }

  HttpPayload payload;
  int n = snprintf(payload.json, sizeof(payload.json),
                   "{\"Tmp_IcnJ1\":\"0\",\"Tmp_DctJ1\":\"0\","
                   "\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\"}",
                   TC_JSON_KEY[0], v[0], TC_JSON_KEY[1], v[1], TC_JSON_KEY[2], v[2],
                   TC_JSON_KEY[3], v[3], TC_JSON_KEY[4], v[4]);
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(payload.json)) {
    logf("[HTTP] payload overflow, upload skipped");
    return;
  }

  if (Config::SERIAL_DEBUG) serialDebugf("[HTTP] enqueue %s", payload.json);
  if (xQueueSend(httpQueue, &payload, 0) != pdTRUE) {
    logf("[HTTP] upload queue full, reading dropped");
  }
}

// ============================================================
//  WIFI
// ============================================================
void WiFi_Config() {
  logf("[WiFi] starting, ssid=%s", Config::WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  varLng_WFiTmeStrChkCnt = millis();
}

void Wifi_Handle(unsigned long now) {
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED && !varBol_WFiCnt) {
    varBol_WFiCnt = true;
    logf("[WiFi] connected, ip=%s rssi=%d", WiFi.localIP().toString().c_str(),
         static_cast<int>(WiFi.RSSI()));
    rjcSend("*O010");
  }
  if (status != WL_CONNECTED && varBol_WFiCnt) {
    varBol_WFiCnt = false;
    ++varU32_WifiReconnects;
    logf("[WiFi] lost, reconnecting");
    rjcSend("*O000");
    // [FIX] v4.4 memanggil WiFi.disconnect(true) (radio dimatikan) lalu
    // begin() di sini sambil auto-reconnect juga berjalan. Sekarang
    // auto-reconnect diberi kesempatan dulu, retry manual tiap 10 s.
    varLng_WFiTmeStrChkCnt = now;
  }
  if (!varBol_WFiCnt && (now - varLng_WFiTmeStrChkCnt >= WIFI_RETRY_PERIOD_MS)) {
    varLng_WFiTmeStrChkCnt = now;
    logf("[WiFi] retry");
    WiFi.disconnect();
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);  // UART0 = bus LFS (dan debug di build bench)

  // Paling awal: image OTA yang gagal sehat 3x boot kembali ke versi lama.
  OtaManager::checkBootRollback();
  beginDebugLog();

  hrdWre_SrlP1_HMI.begin(9600, SERIAL_8N1, pinMCU_SrlRX1, pinMCU_SrlTX1);
  hrdWre_SrlP2_Rjc.begin(9600, SERIAL_8N1, pinMCU_SrlRX2, pinMCU_SrlTX2);

  for (int i = 0; i < NUM_TC; i++) {
    SensorTC &s = sensors[i];
    s.filter.begin();
    s.driver = new Adafruit_MAX31856(TC_CS_PINS[i]);
    if (!s.driver || !s.driver->begin()) {
      s.isOnline = false;
      logf("[INIT] TC%d OFFLINE (CS=%d)", i + 1, TC_CS_PINS[i]);
    } else {
      s.driver->setThermocoupleType(MAX31856_TCTYPE_K);
      s.driver->setConversionMode(MAX31856_ONESHOT_NOWAIT);
      s.isOnline = true;
      logf("[INIT] TC%d OK (CS=%d)", i + 1, TC_CS_PINS[i]);
    }
  }

  while (Serial.available()) Serial.read();
  while (hrdWre_SrlP1_HMI.available()) hrdWre_SrlP1_HMI.read();
  while (hrdWre_SrlP2_Rjc.available()) hrdWre_SrlP2_Rjc.read();

  // EEPROM: limit alarm 5 kanal
  if (!EEPROM.begin(EEPROM_SIZE)) {
    logf("[INIT] EEPROM.begin failed - using default limits (%d)", EEPROM_DEFAULT_LIMIT);
    for (int i = 0; i < NUM_TC; i++) sensors[i].limitValue = EEPROM_DEFAULT_LIMIT;
  } else if (!EEPROMRD_()) {
    logf("[INIT] EEPROM limits invalid - reset to default %d", EEPROM_DEFAULT_LIMIT);
    EEPROMWR_();
  }
  for (int i = 0; i < NUM_TC; i++) _HMISendLimit(i);

  // [FIX] Lampu WiFi di panel bisa tertinggal ON dari sebelum reset.
  rjcSend("*O000");

  httpQueue = xQueueCreate(4, sizeof(HttpPayload));
  if (!httpQueue ||
      xTaskCreatePinnedToCore(Task_HttpPost, "HttpPostTask", 10240, nullptr, 1, nullptr, 0) != pdPASS) {
    logf("[INIT] upload task could not start (out of memory) - uploads disabled");
  }
  if (!Config::apiKeyConfigured()) logf("[INIT] DP_API_KEY not set - the server will answer 401");

  WiFi_Config();
  configTime(conLng_GMTOfs, conInt_DayLghOfs, conChr_SrvNtp);
  beginMqtt();

  logf("[BOOT] ICN2 %s fw %s site %s", Config::DEVICE_ID, Config::FIRMWARE_VERSION, Config::SITE);
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  unsigned long now = millis();

  Wifi_Handle(now);
  Task_SensorRead(now);
  updateAlarmOutputs(now);

  Task_SerialRx(now);
  Task_LFSPoll(now);
  Task_HMIDatetime(now);

  if (now - varLng_DbsTmeStrTmpVal >= SERVER_POST_PERIOD_MS) {
    varLng_DbsTmeStrTmpVal = now;
    Wifi_SendData();
  }

  delay(1);  // beri waktu idle task core 1; UART RX tetap di-buffer driver
}

// ============================================================
//  OUTPUT ALARM KE PANEL
// ============================================================
void updateAlarmOutputs(unsigned long now) {
  // [FIX] Setelah reset, panel bisa masih menyalakan alarm dari sebelum
  // reset padahal ESP32 menganggapnya mati (v4.4 hanya kirim saat berubah).
  // Kirim status sebenarnya sekali, setelah semua sensor sempat membaca.
  if (!varBol_OutputsSynced) {
    bool allReported = true;
    for (int i = 0; i < NUM_TC; i++) {
      if (sensors[i].isOnline && !sensors[i].hasReading && !sensors[i].isFault) allReported = false;
    }
    if (allReported || now >= OUTPUT_SYNC_AFTER_MS) {
      varBol_OutputsSynced = true;
      Bol_TmpTCnTru();
      rjcSend(varBol_TmpTC1OvH ? "*O017" : "*O007");
    }
  }

  bool anyLimit = false;
  for (int i = 0; i < NUM_TC; i++) anyLimit |= sensors[i].limitFlag;
  if (anyLimit != varBol_TmpTCnLmt || varBol_TmpTCn) {
    if (anyLimit != varBol_TmpTCnLmt) logf("[ALARM] limit alarm %s", anyLimit ? "ON" : "OFF");
    varBol_TmpTCnLmt = anyLimit;
    varBol_TmpTCn = false;
    rjcSend(anyLimit ? "*O011*O013" : "*O001*O003");
  }
}

// ============================================================
//  TASK: SENSOR STATE MACHINE
// ============================================================
static void logFaultBits(int n, uint8_t fault) {
  if (fault & MAX31856_FAULT_CJRANGE) logf("[TC%d] fault: cold junction out of range", n);
  if (fault & MAX31856_FAULT_TCRANGE) logf("[TC%d] fault: thermocouple out of range", n);
  if (fault & MAX31856_FAULT_CJHIGH) logf("[TC%d] fault: cold junction temp high", n);
  if (fault & MAX31856_FAULT_CJLOW) logf("[TC%d] fault: cold junction temp low", n);
  if (fault & MAX31856_FAULT_TCHIGH) logf("[TC%d] fault: thermocouple temp high", n);
  if (fault & MAX31856_FAULT_TCLOW) logf("[TC%d] fault: thermocouple temp low", n);
  if (fault & MAX31856_FAULT_OVUV) logf("[TC%d] fault: over/under voltage", n);
  if (fault & MAX31856_FAULT_OPEN) logf("[TC%d] fault: OPEN CIRCUIT - cek kabel", n);
}

// Status alarm (limitFlag) sengaja TIDAK diubah saat fault: alarm yang sedang
// aktif tetap aktif (sama dengan v4.4).
static void markFault(int i, bool timeout, uint8_t faultBits, unsigned long now) {
  SensorTC &s = sensors[i];
  bool wasOk = !s.isFault;
  // [FIX] Log hanya saat status berubah (v4.4 mencetak tiap ~160 ms).
  if (wasOk || s.timedOut != timeout || s.lastFaultBits != faultBits) {
    if (timeout) {
      logf("[TC%d] conversion timeout - MAX31856 not answering (CS=%d)", i + 1, TC_CS_PINS[i]);
    } else {
      logFaultBits(i + 1, faultBits);
    }
  }
  s.isFault = true;
  s.timedOut = timeout;
  s.lastFaultBits = faultBits;
  s.rawValue = SENSOR_FAULT_VALUE;
  s.filteredValue = SENSOR_FAULT_VALUE;
  ++s.faultCount;
  hmiShowSensor(i, now, false);
}

void Task_SensorRead(unsigned long now) {
  for (int i = 0; i < NUM_TC; i++) {
    SensorTC &s = sensors[i];
    if (!s.isOnline) {
      hmiShowSensor(i, now, false);  // tampil 0 merah, refresh 5 s
      continue;
    }

    switch (s.state) {
      case TC_IDLE:
        s.driver->triggerOneShot();
        s.triggerTime = now;
        s.state = TC_TRIGGERED;
        break;

      case TC_TRIGGERED:
        if (now - s.triggerTime >= TC_CONVERSION_TIME_MS) s.state = TC_READY_READ;
        break;

      case TC_READY_READ: {
        if (!s.driver->conversionComplete()) {
          // [FIX] v4.4 menunggu selamanya di sini bila chip tidak menjawab:
          // nilai lama membeku dan tetap di-upload tanpa tanda fault.
          if (now - s.triggerTime >= TC_CONVERSION_TIMEOUT_MS) {
            markFault(i, true, 0, now);
            s.state = TC_IDLE;
          }
          break;
        }

        uint8_t fault = s.driver->readFault();
        if (fault != 0) {
          markFault(i, false, fault, now);
          s.state = TC_IDLE;
          break;
        }

        if (s.isFault) logf("[TC%d] fault cleared", i + 1);
        s.isFault = false;
        s.timedOut = false;
        s.lastFaultBits = 0;

        int raw = static_cast<int>(s.driver->readThermocoupleTemperature());
        int filtered = 0;
        if (!s.filter.addSample(raw, filtered) && Config::SERIAL_DEBUG) {
          serialDebugf("[Filter TC%d] rejected raw=%d, using=%d", i + 1, raw, filtered);
        }
        s.rawValue = raw;
        s.filteredValue = filtered;
        s.hasReading = true;
        s.limitFlag = (filtered >= s.limitValue);

        if (i == 0) {
          bool ovh = (filtered >= TC1_OVERHEAT_LIMIT);
          if (ovh != varBol_TmpTC1OvH) {
            varBol_TmpTC1OvH = ovh;
            rjcSend(ovh ? "*O017" : "*O007");
          }
        }

        hmiShowSensor(i, now, false);
        s.state = TC_IDLE;
        break;
      }
    }
  }
}

// ============================================================
//  TASK: HMI DATETIME
// ============================================================
void Task_HMIDatetime(unsigned long now) {
  if (now - varLng_DMDTmeStrTmpVal < HMI_DATETIME_PERIOD_MS) return;
  varLng_DMDTmeStrTmpVal = now;

  time_t t = time(nullptr);
  struct tm tmNow;
  localtime_r(&t, &tmNow);

  char date[16];
  char clock[16];
  snprintf(date, sizeof(date), "%02d-%02d-%04d", tmNow.tm_mday, tmNow.tm_mon + 1,
           tmNow.tm_year + 1900);
  snprintf(clock, sizeof(clock), "%02d:%02d:%02d", tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);

  hrdWre_SrlP1_HMI.print(F("L_DteTme.txt=\""));
  hrdWre_SrlP1_HMI.print(date);
  hrdWre_SrlP1_HMI.print("  ");
  hrdWre_SrlP1_HMI.print(clock);
  hrdWre_SrlP1_HMI.print("\"");
  HMISfx();
}

// ============================================================
//  TASK: LFS POLL (tidak berubah)
// ============================================================
void Task_LFSPoll(unsigned long now) {
  if (now - varLng_LFSTmeStrGetDta < LFS_POLL_PERIOD_MS) return;
  varLng_LFSTmeStrGetDta = now;

  static const char *const cmds[] = {"#ID:2-GET", "#ID:3-GET", "#ID:4-GET",
                                     "#ID:5-GET", "#ID:6-GET", "#ID:1-GET"};
  static int idx = 0;
  Serial.println(cmds[idx]);
  idx = (idx + 1) % 6;
}

// ============================================================
//  TASK: TERIMA DATA SERIAL (LFS, HMI, panel)
// ============================================================
void Task_SerialRx(unsigned long now) {
  // [FIX] Ketiga jalur sekarang dibatasi panjangnya (v4.4: String tanpa
  // batas, bisa menghabiskan heap), dan satu perintah = satu baris.
  while (Serial.available()) {
    if (lfsLine.feed(static_cast<uint8_t>(Serial.read()))) {
      // [FIX] v4.4 tidak mengosongkan buffer saat halaman Setting aktif,
      // sehingga jawaban LFS menumpuk terus.
      if (varInt_PgeNmb == conInt_PnlLFS || varInt_PgeNmb == conInt_PnlMan) {
        ProcessLFSData(lfsLine.line(), lfsLine.length(), now);
      }
      lfsLine.clear();
    }
  }

  while (hrdWre_SrlP1_HMI.available()) {
    if (hmiParser.feed(static_cast<uint8_t>(hrdWre_SrlP1_HMI.read()))) {
      ProcessHMICommand();
      hmiParser.clear();
    }
  }

  while (hrdWre_SrlP2_Rjc.available()) {
    if (rjcLine.feed(static_cast<uint8_t>(hrdWre_SrlP2_Rjc.read()))) {
      ProcessRjcCommand(rjcLine.line());
      rjcLine.clear();
    }
  }
}

// ============================================================
//  PROSES DATA LFS
// ============================================================
void ProcessLFSData(const char *line, size_t length, unsigned long now) {
  SerialFrames::LfsReading reading;
  // [FIX] v4.4 mengirim "X_VolLFn.val=" kosong ke HMI untuk baris pendek
  // (mis. balasan "#ID:n-RESET").
  if (!SerialFrames::parseLfs(line, length, reading)) {
    ++varU32_LfsRejected;
    return;
  }
  ++varU32_LfsFrames;
  varLng_LfsLastOkMs = now;
  varBol_LfsSeen = true;

  hrdWre_SrlP1_HMI.print("X_VolLF");
  hrdWre_SrlP1_HMI.print(reading.id);
  hrdWre_SrlP1_HMI.print(".val=");
  hrdWre_SrlP1_HMI.print(reading.hmi);
  HMISfx();
}

// ============================================================
//  PROSES COMMAND HMI
// ============================================================
static void handleLimitFrame() {
  int32_t limits[NUM_TC];
  // [FIX] v4.4 menyimpan limit 0 ke EEPROM bila frame terpotong, dan
  // frame terpotong bila byte limit berisi 0x0A/0x0D/0xFF.
  if (!SerialFrames::decodeLimits(hmiParser.limitPayload(), limits)) {
    logf("[EEPROM] limit frame rejected (outside %ld..%ld), keeping current limits",
         static_cast<long>(SerialFrames::LIMIT_MIN), static_cast<long>(SerialFrames::LIMIT_MAX));
    for (int i = 0; i < NUM_TC; i++) _HMISendLimit(i);  // HMI kembali ke nilai tersimpan
    return;
  }

  bool changed = false;
  for (int i = 0; i < NUM_TC; i++) {
    if (sensors[i].limitValue != limits[i]) changed = true;
    sensors[i].limitValue = limits[i];
  }
  if (!changed) return;  // tidak menulis flash untuk nilai yang sama

  EEPROMWR_();
  logf("[EEPROM] limits saved: %d %d %d %d %d", sensors[0].limitValue, sensors[1].limitValue,
       sensors[2].limitValue, sensors[3].limitValue, sensors[4].limitValue);
}

void ProcessHMICommand() {
  if (hmiParser.isLimitFrame()) {
    handleLimitFrame();
    return;
  }

  const char *cmd = hmiParser.command();
  if (hmiParser.length() != 5 || cmd[0] != '*' || cmd[1] != 'I') return;

  // *IBsn : relay n (0..7) off (s=0) / on (s=1)
  if (cmd[2] == 'B' && (cmd[3] == '0' || cmd[3] == '1') && cmd[4] >= '0' && cmd[4] <= '7') {
    RjcRelay(cmd[4] - '0', cmd[3] == '1');
    return;
  }

  // *IR1n : reset volume LFS n+1 (n 0..5)
  if (cmd[2] == 'R' && cmd[3] == '1' && cmd[4] >= '0' && cmd[4] <= '5') {
    int id = cmd[4] - '0' + 1;
    hrdWre_SrlP1_HMI.print("X_VolLF");
    hrdWre_SrlP1_HMI.print(id);
    hrdWre_SrlP1_HMI.print(".val=0");
    HMISfx();
    Serial.print("#ID:");
    Serial.print(id);
    Serial.println("-RESET");
    return;
  }

  // *IP0n : halaman aktif
  if (cmd[2] == 'P' && cmd[3] == '0') {
    if (cmd[4] == '0') varInt_PgeNmb = conInt_PnlMan;
    else if (cmd[4] == '1') varInt_PgeNmb = conInt_PnlLFS;
    else if (cmd[4] == '2') varInt_PgeNmb = conInt_PnlStg;
  }
}

// ============================================================
//  PROSES COMMAND PANEL (REJECTOR)
// ============================================================
void ProcessRjcCommand(const char *cmd) {
  if (strlen(cmd) != 5 || cmd[0] != '*' || cmd[1] != 'I') return;

  // *I11n : tombol relay n (0..7) di panel -> toggle, tampilkan di HMI
  if (cmd[2] == '1' && cmd[3] == '1' && cmd[4] >= '0' && cmd[4] <= '7') {
    int n = cmd[4] - '0';
    bool on = !varBol_RL[n];
    RjcRelay(n, on);
    hrdWre_SrlP1_HMI.print("PANEL_LFSM.B_RL");
    hrdWre_SrlP1_HMI.print(n + 1);
    hrdWre_SrlP1_HMI.print(on ? ".val=1" : ".val=0");
    HMISfx();
    return;
  }

  // *I30n / *I31n : sensor switch n (2..7) aktif / normal -> L_SS(n-1)
  if (cmd[2] == '3' && (cmd[3] == '0' || cmd[3] == '1') && cmd[4] >= '2' && cmd[4] <= '7') {
    hrdWre_SrlP1_HMI.print("PANEL_MAIN.L_SS");
    hrdWre_SrlP1_HMI.print(cmd[4] - '0' - 1);
    hrdWre_SrlP1_HMI.print(cmd[3] == '0' ? ".bco=63488" : ".bco=2024");
    HMISfx();
  }
}

// ============================================================
//  RELAY CONTROL: *O20n = off, *O21n = on (sama dengan v4.4)
// ============================================================
void RjcRelay(int idx, bool on) {
  if (idx < 0 || idx > 7) return;
  char code[6] = {'*', 'O', '2', on ? '1' : '0', static_cast<char>('0' + idx), '\0'};
  rjcSend(code);
  varBol_RL[idx] = on;
}

// ============================================================
//  EEPROM CRC32 (algoritma & layout sama dengan v4.4)
// ============================================================
static uint32_t CRC32_Calculate(const uint8_t *d, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
  }
  return ~crc;
}

void EEPROMWR_() {
  uint8_t raw[EEPROM_DATA_LEN];
  for (int i = 0; i < NUM_TC; i++) {
    uint32_t v = static_cast<uint32_t>(sensors[i].limitValue);
    for (int j = 0; j < 4; j++) raw[i * 4 + j] = static_cast<uint8_t>((v >> (8 * j)) & 0xFF);
  }
  for (int i = 0; i < EEPROM_DATA_LEN; i++) EEPROM.write(i, raw[i]);
  uint32_t crc = CRC32_Calculate(raw, EEPROM_DATA_LEN);
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_CRC_ADDR + i, (crc >> (i * 8)) & 0xFF);
  if (!EEPROM.commit()) logf("[EEPROM] commit failed");
}

// Mengisi sensors[].limitValue. False bila CRC salah atau ada limit di luar
// rentang - limit itu diganti default dan pemanggil menulis ulang EEPROM.
bool EEPROMRD_() {
  uint8_t raw[EEPROM_DATA_LEN];
  for (int i = 0; i < EEPROM_DATA_LEN; i++) raw[i] = EEPROM.read(i);
  uint32_t stored = 0;
  for (int i = 0; i < 4; i++) stored |= static_cast<uint32_t>(EEPROM.read(EEPROM_CRC_ADDR + i)) << (i * 8);

  if (stored != CRC32_Calculate(raw, EEPROM_DATA_LEN)) {
    for (int i = 0; i < NUM_TC; i++) sensors[i].limitValue = EEPROM_DEFAULT_LIMIT;
    return false;
  }

  bool allValid = true;
  for (int i = 0; i < NUM_TC; i++) {
    uint32_t v = static_cast<uint32_t>(raw[i * 4]) | (static_cast<uint32_t>(raw[i * 4 + 1]) << 8) |
                 (static_cast<uint32_t>(raw[i * 4 + 2]) << 16) |
                 (static_cast<uint32_t>(raw[i * 4 + 3]) << 24);
    int32_t limit = static_cast<int32_t>(v);
    // [FIX] limit 0 (dari bug frame terpotong v4.4) = alarm selalu menyala.
    if (!SerialFrames::limitInRange(limit)) {
      logf("[EEPROM] TC%d limit %ld invalid, using default %d", i + 1, static_cast<long>(limit),
           EEPROM_DEFAULT_LIMIT);
      limit = EEPROM_DEFAULT_LIMIT;
      allValid = false;
    }
    sensors[i].limitValue = limit;
  }
  return allValid;
}
