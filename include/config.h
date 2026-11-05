#pragma once

#include <Arduino.h>

// Fleet/network settings for the ICN-2 temperature monitor. The control side
// (pins, thermocouples, HMI, relay panel, LFS meters, alarm limits) lives in
// src/main.cpp; only the network side lives here.
//
// Secrets come from secrets.ini via scripts/load_secrets.py as DP_* macros.
// Nothing secret has a default here: without secrets.ini the uploads and MQTT
// are simply disabled (bench build), and the release env refuses to compile.
namespace Config {

// ------------------------------------------------------------ identity
#ifdef DP_DEVICE_ID
constexpr char DEVICE_ID[] = DP_DEVICE_ID;
#else
constexpr char DEVICE_ID[] = "depok-icn2-01";
#endif

// Fleet this firmware belongs to: MQTT topic prefix ("depok_icn2/<id>/...")
// and the only site whose OTA releases it accepts. Deliberately NOT
// "depok_icn": an ICN-1 image drives four thermocouples on other CS pins and
// a different HMI, and must never be flashed onto this board (or vice versa).
constexpr char SITE[] = "depok_icn2";

// Bumped by hand per release. Reported in the MQTT status; an OTA deployment
// only counts as successful once the device reports the release's version.
// 4.x = the Arduino sketch, 5.x = PlatformIO + fleet/OTA.
constexpr char FIRMWARE_VERSION[] = "5.0.0";

// ------------------------------------------------------------ WiFi
// Unchanged from the v4.4 sketch (same site network as the other Depok units).
#ifdef DP_WIFI_SSID
constexpr char WIFI_SSID[] = DP_WIFI_SSID;
#else
constexpr char WIFI_SSID[] = "Biodigester2025";
#endif
#ifdef DP_WIFI_PASS
constexpr char WIFI_PASSWORD[] = DP_WIFI_PASS;
#else
constexpr char WIFI_PASSWORD[] = "z3r0w4sTe###";
#endif

// ------------------------------------------------------------ HTTP upload
#ifdef DP_SERVER_URL
constexpr char SERVER_URL[] = DP_SERVER_URL;
#else
constexpr char SERVER_URL[] = "https://monitoring.xampah.com/api/tempMonitor2";
#endif
#ifdef DP_API_KEY
constexpr char API_KEY[] = DP_API_KEY;
#else
constexpr char API_KEY[] = "";
#endif

// ------------------------------------------------------------ MQTT
#ifdef DP_MQTT_HOST
constexpr char MQTT_BROKER_HOST[] = DP_MQTT_HOST;
#else
constexpr char MQTT_BROKER_HOST[] = "monitoring.xampah.com";
#endif
#ifdef DP_MQTT_PORT
constexpr uint16_t MQTT_BROKER_PORT = DP_MQTT_PORT;
#else
constexpr uint16_t MQTT_BROKER_PORT = 8883;
#endif
#ifdef DP_MQTT_USER
constexpr char MQTT_USERNAME[] = DP_MQTT_USER;
#else
constexpr char MQTT_USERNAME[] = "";
#endif
#ifdef DP_MQTT_PASS
constexpr char MQTT_PASSWORD[] = DP_MQTT_PASS;
#else
constexpr char MQTT_PASSWORD[] = "";
#endif

// A release/OTA image must never come out unable to reach the server:
// without MQTT it could not receive the next OTA either.
#ifdef DP_RELEASE_BUILD
#if !defined(DP_MQTT_USER) || !defined(DP_MQTT_PASS)
#error "Release build needs DP_MQTT_USER and DP_MQTT_PASS (secrets.ini) - without them the controller loses MQTT and can no longer be updated over the air."
#endif
#if !defined(DP_API_KEY)
#error "Release build needs DP_API_KEY (secrets.ini) - without it every temperature upload is rejected with HTTP 401."
#endif
#endif

// The broker ACL lets username U use depok_icn2/U/..., and the topics are
// built from DEVICE_ID: a second unit with DP_MQTT_USER set but DP_DEVICE_ID
// left at the default would connect and then have every publish dropped.
constexpr bool sameString(const char *a, const char *b) {
  return *a == *b && (*a == '\0' || sameString(a + 1, b + 1));
}
static_assert(MQTT_USERNAME[0] == '\0' || sameString(MQTT_USERNAME, DEVICE_ID),
              "secrets.ini: DP_MQTT_USER must equal DP_DEVICE_ID (set both, e.g. depok-icn2-02)");

constexpr uint32_t MQTT_RECONNECT_MS = 10000;
constexpr uint32_t MQTT_STATUS_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_LOG_DRAIN_INTERVAL_MS = 1500;

#ifdef MQTT_MAX_PACKET_SIZE
constexpr size_t MQTT_PACKET_SIZE = MQTT_MAX_PACKET_SIZE;
#else
constexpr size_t MQTT_PACKET_SIZE = 2048;
#endif

// An OTA image counts as healthy once it published its MQTT status after
// staying up this long (see OtaManager::confirmBootHealthy()).
constexpr uint32_t BOOT_HEALTHY_AFTER_MS = 60000;

// Debug text on UART0. UART0 is shared with the LFS flow meters, so this is
// on for the bench env only (platformio.ini); logf() lines always reach the
// MQTT log either way.
#if defined(DP_SERIAL_DEBUG) && DP_SERIAL_DEBUG
constexpr bool SERIAL_DEBUG = true;
#else
constexpr bool SERIAL_DEBUG = false;
#endif

inline bool apiKeyConfigured() { return API_KEY[0] != '\0'; }
inline bool mqttConfigured() {
  return MQTT_BROKER_HOST[0] != '\0' && MQTT_USERNAME[0] != '\0' && MQTT_PASSWORD[0] != '\0';
}

}  // namespace Config
