#include <unity.h>

#include "sensor_filter.h"

void setUp() {}
void tearDown() {}

void test_first_sample_is_accepted() {
  SensorFilter f;
  f.begin();
  int out = -1;
  TEST_ASSERT_TRUE(f.addSample(500, out));
  TEST_ASSERT_EQUAL(500, out);
}

void test_out_of_range_is_rejected_without_changing_output() {
  SensorFilter f;
  f.begin();
  int out = -1;
  TEST_ASSERT_FALSE(f.addSample(2000, out));
  TEST_ASSERT_EQUAL(0, out);  // nothing accepted yet
  f.addSample(400, out);
  TEST_ASSERT_FALSE(f.addSample(-100, out));
  TEST_ASSERT_EQUAL(400, out);
}

void test_spike_is_rejected_three_times_then_accepted() {
  SensorFilter f;
  f.begin();
  int out = 0;
  for (int i = 0; i < 10; ++i) f.addSample(300, out);
  TEST_ASSERT_FALSE(f.addSample(600, out));
  TEST_ASSERT_FALSE(f.addSample(600, out));
  TEST_ASSERT_FALSE(f.addSample(600, out));
  TEST_ASSERT_EQUAL(300, out);
  TEST_ASSERT_TRUE(f.addSample(600, out));  // a real change gets through
  TEST_ASSERT_TRUE(out > 300);
}

void test_small_steps_track_the_signal() {
  SensorFilter f;
  f.begin();
  int out = 0;
  for (int t = 100; t <= 400; t += 10) TEST_ASSERT_TRUE(f.addSample(t, out));
  TEST_ASSERT_INT_WITHIN(30, 360, out);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_sample_is_accepted);
  RUN_TEST(test_out_of_range_is_rejected_without_changing_output);
  RUN_TEST(test_spike_is_rejected_three_times_then_accepted);
  RUN_TEST(test_small_steps_track_the_signal);
  return UNITY_END();
}
