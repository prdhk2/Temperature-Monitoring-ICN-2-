#include <unity.h>

#include <cstring>

#include "serial_frames.h"

using namespace SerialFrames;

void setUp() {}
void tearDown() {}

namespace {

// Feeds bytes; returns how many complete frames were reported.
template <typename Parser>
int feedAll(Parser &p, const uint8_t *data, size_t n, char last[80] = nullptr) {
  int frames = 0;
  for (size_t i = 0; i < n; ++i) {
    if (p.feed(data[i])) {
      ++frames;
      if (last) {
        memcpy(last, p.command(), p.length());
        last[p.length()] = '\0';
      }
      p.clear();
    }
  }
  return frames;
}

int feedLines(LineAssembler &p, const char *text, char last[80]) {
  int frames = 0;
  for (const char *c = text; *c; ++c) {
    if (p.feed(static_cast<uint8_t>(*c))) {
      ++frames;
      strcpy(last, p.line());
      p.clear();
    }
  }
  return frames;
}

void le32(uint8_t *out, int32_t v) {
  uint32_t u = static_cast<uint32_t>(v);
  out[0] = u & 0xFF;
  out[1] = (u >> 8) & 0xFF;
  out[2] = (u >> 16) & 0xFF;
  out[3] = (u >> 24) & 0xFF;
}

}  // namespace

// ------------------------------------------------------------ LineAssembler

void test_line_cr_terminates_and_lf_is_ignored() {
  LineAssembler p;
  char last[80] = {0};
  TEST_ASSERT_EQUAL(1, feedLines(p, "*I110\r\n", last));
  TEST_ASSERT_EQUAL_STRING("*I110", last);
}

void test_two_commands_in_one_burst_are_both_delivered() {
  // v4.4 glued these into "*I110*I111" and ignored both.
  LineAssembler p;
  char last[80] = {0};
  int frames = 0;
  const char *burst = "*I110\r*I111\r";
  for (const char *c = burst; *c; ++c) {
    if (p.feed(static_cast<uint8_t>(*c))) {
      ++frames;
      if (frames == 1) TEST_ASSERT_EQUAL_STRING("*I110", p.line());
      strcpy(last, p.line());
      p.clear();
    }
  }
  TEST_ASSERT_EQUAL(2, frames);
  TEST_ASSERT_EQUAL_STRING("*I111", last);
}

void test_empty_lines_are_not_reported() {
  LineAssembler p;
  char last[80] = {0};
  TEST_ASSERT_EQUAL(0, feedLines(p, "\r\r\n\r", last));
}

void test_overlong_line_is_discarded_and_parser_recovers() {
  LineAssembler p;
  char last[80] = {0};
  for (int i = 0; i < 500; ++i) TEST_ASSERT_FALSE(p.feed('x'));
  TEST_ASSERT_FALSE(p.feed(CR));  // the overlong line is dropped
  TEST_ASSERT_EQUAL(1u, p.overflows());
  TEST_ASSERT_EQUAL(1, feedLines(p, "*I302\r", last));
  TEST_ASSERT_EQUAL_STRING("*I302", last);
}

// ---------------------------------------------------------------- HmiParser

void test_hmi_text_command() {
  HmiParser p;
  const uint8_t data[] = {'*', 'I', 'P', '0', '2', CR};
  char last[80] = {0};
  TEST_ASSERT_EQUAL(1, feedAll(p, data, sizeof(data), last));
  TEST_ASSERT_EQUAL_STRING("*IP02", last);
}

void test_hmi_ff_resets_partial_command() {
  HmiParser p;
  const uint8_t data[] = {'*', 'I', 0xFF, 0xFF, 0xFF, '*', 'I', 'B', '1', '0', CR};
  char last[80] = {0};
  TEST_ASSERT_EQUAL(1, feedAll(p, data, sizeof(data), last));
  TEST_ASSERT_EQUAL_STRING("*IB10", last);
}

void test_hmi_limit_frame_with_cr_lf_ff_zero_bytes() {
  // 13 = 0x0D (CR), 10 = 0x0A (LF), 255 = 0xFF, 269 = 0x010D, 1023 = 0x03FF.
  const int32_t limits[5] = {13, 10, 255, 269, 1023};
  uint8_t frame[5 + 20 + 1];
  memcpy(frame, "*IL00", 5);
  for (int i = 0; i < 5; ++i) le32(frame + 5 + i * 4, limits[i]);
  frame[25] = CR;

  HmiParser p;
  int frames = 0;
  for (size_t i = 0; i < sizeof(frame); ++i) {
    if (p.feed(frame[i])) {
      ++frames;
      TEST_ASSERT_TRUE(p.isLimitFrame());
      int32_t out[5] = {0};
      TEST_ASSERT_TRUE(decodeLimits(p.limitPayload(), out));
      TEST_ASSERT_EQUAL_INT32_ARRAY(limits, out, 5);
      p.clear();
    }
  }
  TEST_ASSERT_EQUAL(1, frames);  // the trailing CR is an empty line, ignored
}

void test_hmi_command_after_limit_frame_still_parses() {
  uint8_t frame[5 + 20];
  memcpy(frame, "*IL00", 5);
  for (int i = 0; i < 5; ++i) le32(frame + 5 + i * 4, 600);
  HmiParser p;
  for (uint8_t b : frame) p.feed(b);
  p.clear();
  const uint8_t next[] = {CR, '*', 'I', 'P', '0', '0', CR};
  char last[80] = {0};
  TEST_ASSERT_EQUAL(1, feedAll(p, next, sizeof(next), last));
  TEST_ASSERT_EQUAL_STRING("*IP00", last);
}

void test_hmi_overflow_recovers() {
  HmiParser p;
  for (int i = 0; i < 300; ++i) p.feed('A');
  TEST_ASSERT_FALSE(p.feed(CR));
  const uint8_t next[] = {'*', 'I', 'R', '1', '0', CR};
  char last[80] = {0};
  TEST_ASSERT_EQUAL(1, feedAll(p, next, sizeof(next), last));
  TEST_ASSERT_EQUAL_STRING("*IR10", last);
}

// ------------------------------------------------------------------- limits

void test_decode_limits_rejects_zero_and_out_of_range() {
  uint8_t payload[20];
  int32_t out[5] = {7, 7, 7, 7, 7};
  const int32_t good[5] = {7, 7, 7, 7, 7};

  for (int i = 0; i < 5; ++i) le32(payload + i * 4, 500);
  le32(payload + 8, 0);  // a short v4.4 frame used to store 0
  TEST_ASSERT_FALSE(decodeLimits(payload, out));
  TEST_ASSERT_EQUAL_INT32_ARRAY(good, out, 5);  // untouched

  le32(payload + 8, -5);
  TEST_ASSERT_FALSE(decodeLimits(payload, out));
  le32(payload + 8, 1371);
  TEST_ASSERT_FALSE(decodeLimits(payload, out));
  le32(payload + 8, 1370);
  TEST_ASSERT_TRUE(decodeLimits(payload, out));
  TEST_ASSERT_EQUAL_INT32(1370, out[2]);
}

// ---------------------------------------------------------------------- LFS

void test_lfs_parses_volume_digits() {
  //                0123456789012345678901
  const char *l = "#ID:3-DATA:VOL=12.345";
  LfsReading r;
  TEST_ASSERT_TRUE(parseLfs(l, strlen(l), r));
  TEST_ASSERT_EQUAL(3, r.id);
  TEST_ASSERT_EQUAL_STRING("12345", r.hmi);
}

void test_lfs_space_padding_counts_as_zero() {
  const char *l = "#ID:1-DATA:VOL= 2.005";
  LfsReading r;
  TEST_ASSERT_TRUE(parseLfs(l, strlen(l), r));
  TEST_ASSERT_EQUAL_STRING("02005", r.hmi);
}

void test_lfs_rejects_short_and_garbage_lines() {
  LfsReading r;
  const char *reset = "#ID:3-RESET";
  TEST_ASSERT_FALSE(parseLfs(reset, strlen(reset), r));
  const char *badId = "#ID:7-DATA:VOL=12.345";
  TEST_ASSERT_FALSE(parseLfs(badId, strlen(badId), r));
  const char *letters = "#ID:2-DATA:VOL=ab.cde";
  TEST_ASSERT_FALSE(parseLfs(letters, strlen(letters), r));
  const char *blank = "#ID:2-DATA:VOL=  .   ";
  TEST_ASSERT_FALSE(parseLfs(blank, strlen(blank), r));
  const char *noPrefix = "[HTTP] something long enough";
  TEST_ASSERT_FALSE(parseLfs(noPrefix, strlen(noPrefix), r));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_line_cr_terminates_and_lf_is_ignored);
  RUN_TEST(test_two_commands_in_one_burst_are_both_delivered);
  RUN_TEST(test_empty_lines_are_not_reported);
  RUN_TEST(test_overlong_line_is_discarded_and_parser_recovers);
  RUN_TEST(test_hmi_text_command);
  RUN_TEST(test_hmi_ff_resets_partial_command);
  RUN_TEST(test_hmi_limit_frame_with_cr_lf_ff_zero_bytes);
  RUN_TEST(test_hmi_command_after_limit_frame_still_parses);
  RUN_TEST(test_hmi_overflow_recovers);
  RUN_TEST(test_decode_limits_rejects_zero_and_out_of_range);
  RUN_TEST(test_lfs_parses_volume_digits);
  RUN_TEST(test_lfs_space_padding_counts_as_zero);
  RUN_TEST(test_lfs_rejects_short_and_garbage_lines);
  return UNITY_END();
}
