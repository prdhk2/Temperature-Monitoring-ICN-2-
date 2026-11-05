#pragma once

#include <cstddef>

// Pure overflow-handling policy for the MQTT debug-log queue, deliberately
// factored out of debug_log.cpp so it has no FreeRTOS/Arduino dependency and
// can be unit-tested under PlatformIO's native environment (see
// test/test_log_queue_policy/). The queue itself stays a real QueueHandle_t
// in debug_log.cpp - this header only answers "given how full the queue is
// right now, what should happen to a new line".
namespace LogQueuePolicy {

enum class Action {
  ENQUEUE,             // room available, just push
  DROP_OLDEST_THEN_ENQUEUE,  // full: pop one, then push the new line
};

// currentCount/capacity describe the queue's occupancy at the moment a new
// line is about to be enqueued. Oldest-drop, not newest-drop: the newest line
// is the one most likely to explain what just went wrong, so it must never be
// the one silently discarded.
Action decideEnqueueAction(size_t currentCount, size_t capacity);

}  // namespace LogQueuePolicy
