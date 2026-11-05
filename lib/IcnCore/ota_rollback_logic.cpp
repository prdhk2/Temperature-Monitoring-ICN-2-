#include "ota_rollback_logic.h"

namespace OtaRollbackLogic {

BootAction decide(bool runningIsTrial, bool rollbackTargetAvailable, uint8_t trialBoots,
                  uint8_t threshold) {
  if (!runningIsTrial) return BootAction::NOT_ON_TRIAL;
  // With nowhere safe to go, keep booting the new image: a possibly-bad image
  // that runs beats pointing the bootloader at a blank or unknown slot.
  if (rollbackTargetAvailable && trialBoots >= threshold) return BootAction::ROLL_BACK;
  return BootAction::TRIAL_CONTINUE;
}

}  // namespace OtaRollbackLogic
