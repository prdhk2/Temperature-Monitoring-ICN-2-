#include "ota_manager.h"

#include <HTTPClient.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/sha256.h>
#include <strings.h>

#include "config.h"
#include "debug_log.h"
#include "mqtt_ca_cert.h"
#include "ota_rollback_logic.h"
#include "reset_reason.h"

#include <esp_system.h>

namespace OtaManager {

namespace {

constexpr uint32_t TOTAL_TIMEOUT_MS = 90000;
constexpr uint32_t RETRY_DELAY_MS = 5000;
constexpr size_t CHUNK_SIZE = 1024;

// Progress goes out at most once a second and only after moving 2 %: a
// smooth dashboard bar, and the publishes double as MQTT keep-alive while
// this task is too busy downloading to run client.loop().
constexpr uint32_t PROGRESS_INTERVAL_MS = 1000;
constexpr int PROGRESS_MIN_STEP_PCT = 2;

constexpr const char *NVS_NAMESPACE = "dpota";
constexpr const char *KEY_TRIAL_PART = "trialpart";  // partition OTA flashed, not yet confirmed
constexpr const char *KEY_TRIAL_BOOTS = "trialboots";
constexpr const char *KEY_GOOD_PART = "goodpart";  // last partition that confirmed itself
// Written when a rollback happens, read (and cleared) by takeRollbackNote()
// on the restored image: the rollback restart itself is invisible remotely.
constexpr const char *KEY_RB_FROM = "rbfrom";
constexpr const char *KEY_RB_BOOTS = "rbboots";
constexpr const char *KEY_RB_RESET = "rbreset";

ProgressFn progressFn = nullptr;
char lastError[64] = "";
bool runningOnTrial = false;
volatile bool updating = false;

void report(const char *stage, int pct, const char *detail = "") {
  if (progressFn) progressFn(stage, pct, detail);
}

void setError(const char *format, ...) __attribute__((format(printf, 1, 2)));
void setError(const char *format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(lastError, sizeof(lastError), format, args);
  va_end(args);
  for (char *c = lastError; *c; ++c) {
    if (*c == '"' || *c == '\\') *c = '\'';
  }
}

void bytesToHex(const uint8_t *bytes, size_t count, char *out) {
  static const char digits[] = "0123456789abcdef";
  for (size_t i = 0; i < count; ++i) {
    out[i * 2] = digits[(bytes[i] >> 4) & 0x0F];
    out[i * 2 + 1] = digits[bytes[i] & 0x0F];
  }
  out[count * 2] = '\0';
}

bool isHexSha256(const char *s) {
  if (strlen(s) != 64) return false;
  for (const char *c = s; *c; ++c) {
    bool hex = (*c >= '0' && *c <= '9') || (*c >= 'a' && *c <= 'f') || (*c >= 'A' && *c <= 'F');
    if (!hex) return false;
  }
  return true;
}

// Records the freshly flashed partition as on trial, and the running one -
// which just proved itself by receiving this command over MQTT - as the
// rollback target.
void startTrial(const esp_partition_t *flashed) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, false);
  if (flashed) prefs.putString(KEY_TRIAL_PART, flashed->label);
  prefs.putUChar(KEY_TRIAL_BOOTS, 0);
  if (running) prefs.putString(KEY_GOOD_PART, running->label);
  prefs.end();
}

// One attempt: stream the image into the idle OTA partition while hashing it,
// and only finalise the write if the digest matches.
bool attemptOnce(const Command &command, uint32_t deadlineMs) {
  WiFiClientSecure secureClient;
  WiFiClient plainClient;
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(20000);

  bool https = strncmp(command.url, "https://", 8) == 0;
  bool begun;
  if (https) {
    secureClient.setCACert(ISRG_ROOT_X1_PEM);
    begun = http.begin(secureClient, command.url);
  } else {
    begun = http.begin(plainClient, command.url);
  }
  if (!begun) {
    logf("[OTA] http.begin failed for %.80s", command.url);
    setError("bad download url");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    logf("[OTA] download failed, http=%d", code);
    setError(code < 0 ? "connection/TLS failed (http=%d)" : "download failed (http=%d)", code);
    http.end();
    return false;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0) {
    logf("[OTA] server did not report a content length");
    setError("no content length from server");
    http.end();
    return false;
  }

  if (!Update.begin(static_cast<size_t>(contentLength), U_FLASH)) {
    logf("[OTA] Update.begin failed: %s", Update.errorString());
    setError("image does not fit OTA partition (%d bytes)", contentLength);
    http.end();
    return false;
  }

  report("downloading", 0);
  uint32_t lastReportMs = millis();
  int lastReportPct = 0;

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts_ret(&sha, 0);  // 0 = SHA-256, not SHA-224

  WiFiClient *stream = http.getStreamPtr();
  static uint8_t buffer[CHUNK_SIZE];  // this task only; kept off its stack
  int written = 0;
  bool timedOut = false;
  bool writeFailed = false;

  while (written < contentLength) {
    if (static_cast<int32_t>(millis() - deadlineMs) > 0) {
      timedOut = true;
      break;
    }
    if (!http.connected() && stream->available() == 0) break;

    size_t avail = stream->available();
    if (avail == 0) {
      delay(2);
      continue;
    }
    size_t remaining = static_cast<size_t>(contentLength - written);
    size_t toRead = avail < sizeof(buffer) ? avail : sizeof(buffer);
    if (toRead > remaining) toRead = remaining;
    int readCount = stream->readBytes(buffer, toRead);
    if (readCount <= 0) continue;

    if (Update.write(buffer, static_cast<size_t>(readCount)) != static_cast<size_t>(readCount)) {
      writeFailed = true;
      break;
    }
    mbedtls_sha256_update_ret(&sha, buffer, static_cast<size_t>(readCount));
    written += readCount;

    int pct = static_cast<int>((static_cast<int64_t>(written) * 100) / contentLength);
    if (millis() - lastReportMs >= PROGRESS_INTERVAL_MS && pct - lastReportPct >= PROGRESS_MIN_STEP_PCT) {
      report("downloading", pct);
      lastReportMs = millis();
      lastReportPct = pct;
    }
  }
  http.end();

  uint8_t digest[32];
  mbedtls_sha256_finish_ret(&sha, digest);
  mbedtls_sha256_free(&sha);

  if (writeFailed) {
    logf("[OTA] flash write failed: %s", Update.errorString());
    setError("flash write failed: %s", Update.errorString());
    Update.abort();
    return false;
  }
  if (timedOut || written != contentLength) {
    int pct = static_cast<int>((static_cast<int64_t>(written) * 100) / contentLength);
    logf("[OTA] download incomplete (%d/%d bytes, timed_out=%s)", written, contentLength,
         timedOut ? "yes" : "no");
    setError(timedOut ? "download timed out at %d%%" : "connection dropped at %d%%", pct);
    Update.abort();
    return false;
  }

  report("verifying", 100);

  char digestHex[65];
  bytesToHex(digest, sizeof(digest), digestHex);
  if (strcasecmp(digestHex, command.sha256) != 0) {
    logf("[OTA] checksum mismatch, expected=%.16s got=%.16s", command.sha256, digestHex);
    setError("checksum mismatch - image corrupted or wrong file");
    Update.abort();
    return false;
  }

  if (!Update.end(true)) {
    logf("[OTA] Update.end failed: %s", Update.errorString());
    setError("flash finalize failed: %s", Update.errorString());
    return false;
  }

  // Update.end() already pointed the bootloader at the new partition.
  startTrial(esp_ota_get_boot_partition());

  report("rebooting", 100, command.version);
  logf("[OTA] flashed version %s ok (%d bytes, sha256 verified), rebooting", command.version,
       written);
  return true;
}

void rebootAfterSuccess() {
  delay(500);  // let the "rebooting" report leave the socket
  ESP.restart();
}

}  // namespace

static bool applyUpdateImpl(const Command &command, ProgressFn progress) {
  progressFn = progress;
  lastError[0] = '\0';
  logf("[OTA] update requested: version=%s", command.version);
  report("accepted", 0, command.version);

  if (!isHexSha256(command.sha256)) {
    logf("[OTA] rejected: sha256 is not 64 hex characters");
    report("failed", -1, "invalid sha256 in command");
    progressFn = nullptr;
    return false;
  }

  uint32_t deadline = millis() + TOTAL_TIMEOUT_MS;
  if (attemptOnce(command, deadline)) {
    rebootAfterSuccess();
    return true;  // unreachable
  }

  if (static_cast<int32_t>(millis() - deadline) > 0) {
    logf("[OTA] giving up, total timeout exceeded");
    report("failed", -1, lastError[0] ? lastError : "total timeout exceeded");
    progressFn = nullptr;
    return false;
  }

  logf("[OTA] first attempt failed, retrying once in %lus",
       static_cast<unsigned long>(RETRY_DELAY_MS / 1000UL));
  report("retrying", -1, lastError);
  delay(RETRY_DELAY_MS);

  if (attemptOnce(command, deadline)) {
    rebootAfterSuccess();
    return true;  // unreachable
  }

  logf("[OTA] update failed after retry, staying on current firmware");
  report("failed", -1, lastError[0] ? lastError : "unknown error");
  progressFn = nullptr;
  return false;
}

bool applyUpdate(const Command &command, ProgressFn progress) {
  // While set, the upload task holds back: the download, the MQTT session
  // and an upload would otherwise be three TLS sessions at once (heap).
  updating = true;
  bool ok = applyUpdateImpl(command, progress);
  updating = false;  // only reached on failure; success reboots
  return ok;
}

bool inProgress() { return updating; }

void checkBootRollback() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (!running) return;

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return;

  String trialLabel = prefs.getString(KEY_TRIAL_PART, "");
  bool isTrial = trialLabel.length() > 0 && trialLabel == running->label;

  if (!isTrial) {
    // A USB-flashed or already-confirmed image: nothing on trial. Clear any
    // leftover bookkeeping (e.g. an OTA whose new image never booted).
    if (trialLabel.length() > 0) prefs.remove(KEY_TRIAL_PART);
    prefs.end();
    runningOnTrial = false;
    return;
  }

  uint8_t boots = prefs.getUChar(KEY_TRIAL_BOOTS, 0);
  boots = boots < 255 ? static_cast<uint8_t>(boots + 1) : 255;
  prefs.putUChar(KEY_TRIAL_BOOTS, boots);

  String goodLabel = prefs.getString(KEY_GOOD_PART, "");
  const esp_partition_t *target = nullptr;
  if (goodLabel.length() > 0 && goodLabel != running->label) {
    target = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY,
                                      goodLabel.c_str());
  }

  OtaRollbackLogic::BootAction action =
      OtaRollbackLogic::decide(true, target != nullptr, boots);

  if (action == OtaRollbackLogic::BootAction::ROLL_BACK &&
      esp_ota_set_boot_partition(target) == ESP_OK) {
    prefs.remove(KEY_TRIAL_PART);
    prefs.putUChar(KEY_TRIAL_BOOTS, 0);
    // This boot's reset reason is why the last trial boot ended (crash,
    // watchdog, brownout...): the most useful clue to why the image failed.
    prefs.putString(KEY_RB_FROM, running->label);
    prefs.putUChar(KEY_RB_BOOTS, boots);
    prefs.putInt(KEY_RB_RESET, static_cast<int>(esp_reset_reason()));
    prefs.end();
    serialDebugf("[OTA] image on %s never confirmed after %u boots, rolling back to %s",
                 running->label, boots, target->label);
    if (Config::SERIAL_DEBUG) Serial.flush();
    ESP.restart();
    return;  // unreachable
  }

  prefs.end();
  runningOnTrial = true;
  serialDebugf("[OTA] new image on trial, boot %u/%u", boots,
               OtaRollbackLogic::DEFAULT_ROLLBACK_THRESHOLD);
}

void confirmBootHealthy() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (!running) return;

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return;
  // Written only when something changes: this runs once every boot, and
  // NVS writes wear flash.
  if (prefs.getString(KEY_GOOD_PART, "") != running->label) prefs.putString(KEY_GOOD_PART, running->label);
  if (prefs.isKey(KEY_TRIAL_PART)) prefs.remove(KEY_TRIAL_PART);
  if (prefs.getUChar(KEY_TRIAL_BOOTS, 0) != 0) prefs.putUChar(KEY_TRIAL_BOOTS, 0);
  prefs.end();

  if (runningOnTrial) {
    runningOnTrial = false;
    logf("[OTA] new image confirmed healthy on %s", running->label);
  }
}

bool onTrial() { return runningOnTrial; }

const char *runningPartition() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  return running ? running->label : "?";
}

bool takeRollbackNote(char *out, size_t size) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  if (!prefs.isKey(KEY_RB_FROM)) {
    prefs.end();
    return false;
  }
  String from = prefs.getString(KEY_RB_FROM, "?");
  uint8_t boots = prefs.getUChar(KEY_RB_BOOTS, 0);
  int reset = prefs.getInt(KEY_RB_RESET, 0);
  prefs.remove(KEY_RB_FROM);
  prefs.remove(KEY_RB_BOOTS);
  prefs.remove(KEY_RB_RESET);
  prefs.end();

  snprintf(out, size, "[OTA] rolled back: new image on %s never confirmed in %u boots (last reset=%s), now on %s",
           from.c_str(), boots, ResetReason::name(reset), runningPartition());
  return true;
}

}  // namespace OtaManager
