#include <unity.h>

#include "ota_rollback_logic.h"

using OtaRollbackLogic::BootAction;
using OtaRollbackLogic::decide;
using OtaRollbackLogic::DEFAULT_ROLLBACK_THRESHOLD;

void setUp() {}
void tearDown() {}

void test_image_not_on_trial_is_never_rolled_back() {
  // USB-flashed or already confirmed: however many boots, whatever is in the
  // other slot.
  TEST_ASSERT_EQUAL(BootAction::NOT_ON_TRIAL, decide(false, true, 0));
  TEST_ASSERT_EQUAL(BootAction::NOT_ON_TRIAL, decide(false, true, 200));
  TEST_ASSERT_EQUAL(BootAction::NOT_ON_TRIAL, decide(false, false, 200));
}

void test_trial_image_gets_threshold_boots() {
  for (uint8_t boots = 1; boots < DEFAULT_ROLLBACK_THRESHOLD; ++boots) {
    TEST_ASSERT_EQUAL(BootAction::TRIAL_CONTINUE, decide(true, true, boots));
  }
  TEST_ASSERT_EQUAL(BootAction::ROLL_BACK, decide(true, true, DEFAULT_ROLLBACK_THRESHOLD));
  // A boot loop that keeps counting keeps failing the check.
  TEST_ASSERT_EQUAL(BootAction::ROLL_BACK, decide(true, true, 255));
}

void test_no_target_means_keep_running() {
  // Nothing confirmed to go back to: never point the bootloader at a blank slot.
  TEST_ASSERT_EQUAL(BootAction::TRIAL_CONTINUE, decide(true, false, DEFAULT_ROLLBACK_THRESHOLD));
  TEST_ASSERT_EQUAL(BootAction::TRIAL_CONTINUE, decide(true, false, 255));
}

void test_custom_threshold() {
  TEST_ASSERT_EQUAL(BootAction::TRIAL_CONTINUE, decide(true, true, 1, 2));
  TEST_ASSERT_EQUAL(BootAction::ROLL_BACK, decide(true, true, 2, 2));
}

int runAllTests() {
  UNITY_BEGIN();
  RUN_TEST(test_image_not_on_trial_is_never_rolled_back);
  RUN_TEST(test_trial_image_gets_threshold_boots);
  RUN_TEST(test_no_target_means_keep_running);
  RUN_TEST(test_custom_threshold);
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
