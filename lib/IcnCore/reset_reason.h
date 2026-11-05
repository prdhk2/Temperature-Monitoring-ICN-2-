#pragma once

#include <cstdint>

// Short names for ESP-IDF's esp_reset_reason_t, factored out of mqtt_client.cpp
// so the mapping has no ESP-IDF dependency and can be unit-tested under
// PlatformIO's native environment (see test/test_reset_reason/).
//
// Published in the MQTT status payload ("reset":"BROWNOUT") so the dashboard
// can say WHY a gateway rebooted - a brownout every time a pump starts reads
// completely differently from a watchdog after a firmware hang.
//
// The numeric values mirror esp_reset_reason_t (esp_system.h) and are part of
// ESP-IDF's stable API; mqtt_client.cpp static_asserts the ones it relies on.
namespace ResetReason {

enum Code : int {
  UNKNOWN = 0,
  POWERON = 1,
  EXT = 2,
  SW = 3,
  PANIC = 4,
  INT_WDT = 5,
  TASK_WDT = 6,
  WDT = 7,
  DEEPSLEEP = 8,
  BROWNOUT = 9,
  SDIO = 10,
};

// "POWERON", "BROWNOUT", ... ; "UNKNOWN" for anything unrecognised.
const char *name(int code);

// True for resets that mean something went wrong (crash, watchdog, supply
// sag) rather than a deliberate power-on, restart command or OTA reboot.
bool isAbnormal(int code);

}  // namespace ResetReason
