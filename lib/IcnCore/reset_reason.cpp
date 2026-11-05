#include "reset_reason.h"

namespace ResetReason {

const char *name(int code) {
  switch (code) {
    case POWERON: return "POWERON";
    case EXT: return "EXT";
    case SW: return "SW";
    case PANIC: return "PANIC";
    case INT_WDT: return "INT_WDT";
    case TASK_WDT: return "TASK_WDT";
    case WDT: return "WDT";
    case DEEPSLEEP: return "DEEPSLEEP";
    case BROWNOUT: return "BROWNOUT";
    case SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

bool isAbnormal(int code) {
  switch (code) {
    case PANIC:
    case INT_WDT:
    case TASK_WDT:
    case WDT:
    case BROWNOUT:
      return true;
    default:
      return false;
  }
}

}  // namespace ResetReason
