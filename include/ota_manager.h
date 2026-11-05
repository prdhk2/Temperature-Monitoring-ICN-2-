#pragma once

#include <Arduino.h>

// Over-the-air firmware update, triggered by the MQTT "OTA" command.
//
// applyUpdate() downloads the image over HTTPS, computes its SHA-256 while
// streaming it into the idle app partition, and only lets that partition
// become bootable if the digest matches the one the server sent. Any failure
// (bad hash, timeout, flash error) leaves the running firmware untouched.
// Runs on the MQTT task (core 0), never on the watchdog-registered loop task,
// so the display and serial links keep running during the download.
//
// The safety net against a bad build is the boot trial (checkBootRollback /
// confirmBootHealthy, see ota_rollback_logic.h): an OTA image that never gets
// itself onto MQTT within three boots is replaced by the previous one.
namespace OtaManager {

struct Command {
  char url[192];
  char sha256[65];  // 64 hex chars + NUL
  char version[32];
};

// Progress for the dashboard. stage: "accepted", "downloading", "retrying",
// "verifying", "rebooting", "failed". pct: 0..100, or -1 when not applicable.
// detail: short reason or version, never containing '"' or '\'.
using ProgressFn = void (*)(const char *stage, int pct, const char *detail);

// Blocks the calling task for up to ~90 s. On success the device reboots and
// this never returns; on failure it returns false.
bool applyUpdate(const Command &command, ProgressFn progress = nullptr);

// First thing in setup(). Counts boots of an OTA image that has not yet
// confirmed itself; after too many, boots the last confirmed partition
// (does not return in that case).
void checkBootRollback();

// Once per boot, when the image has proven itself (MQTT status published
// after Config::BOOT_HEALTHY_AFTER_MS of uptime). Ends any boot trial and
// records the running partition as the rollback target for future updates.
void confirmBootHealthy();

// True while applyUpdate() is downloading/flashing (MQTT task). The upload
// task skips its POSTs meanwhile to keep heap for the download's TLS session.
bool inProgress();

// True while the running image is an OTA image still on trial.
bool onTrial();

// Label of the app partition this image runs from ("app0" / "app1"),
// reported in the status so a rollback is visible at a glance.
const char *runningPartition();

// If the previous boot rolled an unconfirmed OTA image back (see
// checkBootRollback()), writes one log line describing it - which slot, how
// many boots, and the reset reason of the last failed one - clears the note
// and returns true. Call once MQTT is up, so the line reaches the dashboard.
bool takeRollbackNote(char *out, size_t size);

}  // namespace OtaManager
