#include "log_queue_policy.h"

namespace LogQueuePolicy {

Action decideEnqueueAction(size_t currentCount, size_t capacity) {
  if (currentCount < capacity) return Action::ENQUEUE;
  return Action::DROP_OLDEST_THEN_ENQUEUE;
}

}  // namespace LogQueuePolicy
