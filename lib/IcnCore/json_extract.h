#pragma once

#include <cstddef>
#include <cstdint>

// Flat-JSON field extraction for the MQTT cmd payloads (mqtt_client.cpp).
// Deliberately non-general (no ArduinoJson): the only producer is our own
// Laravel backend publishing a fixed, flat shape. Pure C++, unit-tested natively in
// test/test_json_extract/.
namespace JsonExtract {

// Copies the string value of "key" into out, decoding the JSON escapes a
// real encoder may emit (\/ \ \" \n \t \r \b \f). PHP's json_encode()
// escapes every '/' by default - an OTA url copied verbatim arrived as
// "https:\/\/host\/..." and broke HTTPClient's host parsing. Returns false
// if the key is missing, the value is not a string, the string is
// unterminated, it uses \u escapes, or it does not fit in outSize - never
// a silently truncated value.
bool string(const char *json, const char *key, char *out, size_t outSize);

// Unsigned decimal only. Returns false if the key is missing or its value is
// not a plain digit run (a string, negative number, float or overflow).
bool uint(const char *json, const char *key, uint32_t &out);

// True if "key" is present with any value.
bool hasKey(const char *json, const char *key);

}  // namespace JsonExtract
