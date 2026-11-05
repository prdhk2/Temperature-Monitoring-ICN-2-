#include <unity.h>

#include "log_queue_policy.h"

using LogQueuePolicy::Action;
using LogQueuePolicy::decideEnqueueAction;

void setUp() {}
void tearDown() {}

void test_enqueue_when_room_available() {
  TEST_ASSERT_EQUAL(static_cast<int>(Action::ENQUEUE),
                     static_cast<int>(decideEnqueueAction(0, 32)));
  TEST_ASSERT_EQUAL(static_cast<int>(Action::ENQUEUE),
                     static_cast<int>(decideEnqueueAction(31, 32)));
}

void test_drop_oldest_when_full() {
  TEST_ASSERT_EQUAL(static_cast<int>(Action::DROP_OLDEST_THEN_ENQUEUE),
                     static_cast<int>(decideEnqueueAction(32, 32)));
}

void test_drop_oldest_when_over_capacity() {
  // Defensive: should never happen with a real QueueHandle_t, but the policy
  // must not treat "somehow over capacity" as "there's room".
  TEST_ASSERT_EQUAL(static_cast<int>(Action::DROP_OLDEST_THEN_ENQUEUE),
                     static_cast<int>(decideEnqueueAction(33, 32)));
}

void test_zero_capacity_always_drops() {
  TEST_ASSERT_EQUAL(static_cast<int>(Action::DROP_OLDEST_THEN_ENQUEUE),
                     static_cast<int>(decideEnqueueAction(0, 0)));
}

int runAllTests() {
  UNITY_BEGIN();
  RUN_TEST(test_enqueue_when_room_available);
  RUN_TEST(test_drop_oldest_when_full);
  RUN_TEST(test_drop_oldest_when_over_capacity);
  RUN_TEST(test_zero_capacity_always_drops);
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
