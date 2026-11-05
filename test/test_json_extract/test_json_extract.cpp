#include <unity.h>

#include <cstdio>
#include <cstring>

#include "json_extract.h"

void setUp() {}
void tearDown() {}

// Byte-for-byte what the backend's old plain json_encode() published - the
// payload that made every OTA fail with http=-1 on firmware 1.0.2.
static const char *ESCAPED_OTA =
    "{\"cmd\":\"OTA\",\"url\":\"https:\\/\\/monitoring.xampah.com\\/storage\\/firmware\\/"
    "6585403d628d9f61028f840fa3cc9c3b8b9326631b6d082782f6971ca5da8d1d.bin\","
    "\"sha256\":\"6585403d628d9f61028f840fa3cc9c3b8b9326631b6d082782f6971ca5da8d1d\","
    "\"version\":\"1.1.0\"}";

static const char *CLEAN_URL =
    "https://monitoring.xampah.com/storage/firmware/"
    "6585403d628d9f61028f840fa3cc9c3b8b9326631b6d082782f6971ca5da8d1d.bin";

void test_escaped_slashes_are_decoded() {
  char url[192];
  TEST_ASSERT_TRUE(JsonExtract::string(ESCAPED_OTA, "url", url, sizeof(url)));
  TEST_ASSERT_EQUAL_STRING(CLEAN_URL, url);
}

void test_unescaped_payload_still_reads_the_same() {
  char json[400];
  snprintf(json, sizeof(json), "{\"cmd\":\"OTA\",\"url\":\"%s\",\"version\":\"1.1.0\"}", CLEAN_URL);
  char url[192];
  TEST_ASSERT_TRUE(JsonExtract::string(json, "url", url, sizeof(url)));
  TEST_ASSERT_EQUAL_STRING(CLEAN_URL, url);
}

void test_other_ota_fields() {
  char sha[65];
  char version[32];
  char cmd[16];
  TEST_ASSERT_TRUE(JsonExtract::string(ESCAPED_OTA, "sha256", sha, sizeof(sha)));
  TEST_ASSERT_EQUAL_UINT(64, strlen(sha));
  TEST_ASSERT_TRUE(JsonExtract::string(ESCAPED_OTA, "version", version, sizeof(version)));
  TEST_ASSERT_EQUAL_STRING("1.1.0", version);
  TEST_ASSERT_TRUE(JsonExtract::string(ESCAPED_OTA, "cmd", cmd, sizeof(cmd)));
  TEST_ASSERT_EQUAL_STRING("OTA", cmd);
}

void test_escaped_quote_and_backslash() {
  char out[32];
  // JSON text: {"k":"a\"b\\c"}  ->  a"b\c
  TEST_ASSERT_TRUE(JsonExtract::string("{\"k\":\"a\\\"b\\\\c\"}", "k", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("a\"b\\c", out);
}

void test_never_silently_truncates() {
  char out[4];
  TEST_ASSERT_FALSE(JsonExtract::string("{\"k\":\"abcd\"}", "k", out, sizeof(out)));
  TEST_ASSERT_TRUE(JsonExtract::string("{\"k\":\"abc\"}", "k", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("abc", out);
}

void test_rejects_unterminated_missing_and_non_string() {
  char out[32];
  // A payload cut off by a too-small receive buffer.
  TEST_ASSERT_FALSE(JsonExtract::string("{\"url\":\"https:\\/\\/monitor", "url", out, sizeof(out)));
  TEST_ASSERT_FALSE(JsonExtract::string("{\"a\":\"b\"}", "url", out, sizeof(out)));
  TEST_ASSERT_FALSE(JsonExtract::string("{\"url\":42}", "url", out, sizeof(out)));
  TEST_ASSERT_FALSE(JsonExtract::string("{\"k\":\"\\u0041\"}", "k", out, sizeof(out)));
}

int runAllTests() {
  UNITY_BEGIN();
  RUN_TEST(test_escaped_slashes_are_decoded);
  RUN_TEST(test_unescaped_payload_still_reads_the_same);
  RUN_TEST(test_other_ota_fields);
  RUN_TEST(test_escaped_quote_and_backslash);
  RUN_TEST(test_never_silently_truncates);
  RUN_TEST(test_rejects_unterminated_missing_and_non_string);
  return UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
  delay(2000);  // let the serial monitor attach
  runAllTests();
}
void loop() {}
#else
int main() { return runAllTests(); }
#endif
