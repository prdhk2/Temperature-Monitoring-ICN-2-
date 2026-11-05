#pragma once

#include <Arduino.h>

// MQTT link to the fleet broker (monitoring.xampah.com:8883, TLS), on its own
// FreeRTOS task pinned to core 0 - loop() (thermocouples, HMI, relay panel,
// LFS meters, alarms) keeps running on core 1, also while an OTA downloads.
//
// Topics (prefix = Config::SITE, id = Config::DEVICE_ID):
//   depok_icn2/<id>/status  publish, retained, LWT {"online":false}:
//                           firmware, 5 filtered temperatures with their
//                           limits and health, alarm outputs, LFS link,
//                           upload counters (see buildStatus())
//   depok_icn2/<id>/log     publish, batched debug_log lines + OTA progress
//   depok_icn2/<id>/cmd     subscribe:
//                             {"cmd":"PING"|"STATUS"|"RESTART"}
//                             {"cmd":"OTA","url":..,"sha256":..,"version":..,"site":"depok_icn2"}
void beginMqtt();

bool mqttConnected();
uint32_t mqttReconnects();

// Snapshot of the controller state for the status payload. Filled by
// main.cpp (icn2Status()), called from the MQTT task; only plain ints/bools
// are copied, each of them written by a single task.
constexpr int ICN2_CHANNELS = 5;

enum class TcHealth : uint8_t {
  OK,       // valid reading
  WAIT,     // no reading since boot yet
  FAULT,    // MAX31856 fault (open circuit, out of range, ...)
  TIMEOUT,  // conversion never completed (chip not answering)
  OFFLINE,  // chip failed to initialise at boot
};

struct Icn2Status {
  const char *key[ICN2_CHANNELS];  // temp_monitor2 column, e.g. "Tmp_CmbJ1"
  int temp[ICN2_CHANNELS];         // filtered degrees C (0 when not OK)
  int limit[ICN2_CHANNELS];        // alarm limit (EEPROM)
  TcHealth health[ICN2_CHANNELS];
  uint32_t faults[ICN2_CHANNELS];  // fault/timeout readings since boot
  bool alarm;                      // limit alarm output on the panel
  bool overheat;                   // TC1 over-heat output on the panel
  uint32_t lfsFrames, lfsRejected;
  long lfsAgeS;                    // -1 = no valid LFS answer since boot
  uint32_t uploadsOk, uploadsFailed;
  int lastHttp;                    // 0 = none yet, < 0 = transport error
  unsigned long lastUploadOkEpoch; // 0 = none
  uint32_t wifiReconnects;
};
Icn2Status icn2Status();
