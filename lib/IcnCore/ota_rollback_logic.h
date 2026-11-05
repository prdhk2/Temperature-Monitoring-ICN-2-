#pragma once

#include <cstdint>

// Decision logic for the OTA boot-trial safety net. Pure C++ (no NVS or
// esp_ota), unit-tested natively in test/test_ota_rollback_logic/.
//
// Only an image that arrived by OTA is ever "on trial". OtaManager records the
// partition it just flashed; every boot of that partition counts, and once it
// has booted `threshold` times without ever confirming itself healthy (MQTT
// status published after a minute of uptime), the device boots back into the
// last confirmed partition.
//
// Deliberately NOT on trial: an image flashed over USB, and a confirmed image
// that merely power-cycles. A technician's bench build without MQTT secrets,
// or a site whose internet is down through several power cuts, must never be
// "rolled back" to some older image sitting in the other slot.
namespace OtaRollbackLogic {

constexpr uint8_t DEFAULT_ROLLBACK_THRESHOLD = 3;

enum class BootAction : uint8_t {
  NOT_ON_TRIAL,    // nothing to do; clear any leftover trial bookkeeping
  TRIAL_CONTINUE,  // keep running the new image and keep counting
  ROLL_BACK,       // boot the last confirmed partition now
};

// runningIsTrial: the running partition is the one OTA last flashed and it has
// not confirmed itself yet. trialBoots: boots of it so far, this one included.
// rollbackTargetAvailable: a different, previously confirmed partition exists.
BootAction decide(bool runningIsTrial, bool rollbackTargetAvailable, uint8_t trialBoots,
                  uint8_t threshold = DEFAULT_ROLLBACK_THRESHOLD);

}  // namespace OtaRollbackLogic
