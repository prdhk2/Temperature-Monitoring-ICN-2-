#include "sensor_filter.h"

#include <cstdlib>
#include <cstring>

void SensorFilter::begin() {
  memset(window_, 0, sizeof(window_));
  windowIndex_ = 0;
  windowCount_ = 0;
  lastAccepted_ = 0;
  spikeCount_ = 0;
  initialized_ = false;
}

bool SensorFilter::addSample(int raw, int &out) {
  if (raw < MIN_VALID_TEMP || raw > MAX_VALID_TEMP) {
    out = initialized_ ? filtered() : 0;
    return false;
  }
  if (initialized_) {
    if (abs(raw - lastAccepted_) > SPIKE_THRESHOLD) {
      ++spikeCount_;
      if (spikeCount_ <= SPIKE_MAX_CONSECUTIVE) {
        out = filtered();
        return false;
      }
    }
    spikeCount_ = 0;
  }
  window_[windowIndex_] = raw;
  windowIndex_ = (windowIndex_ + 1) % WINDOW_SIZE;
  if (windowCount_ < WINDOW_SIZE) ++windowCount_;
  lastAccepted_ = raw;
  initialized_ = true;
  out = filtered();
  return true;
}

int SensorFilter::median() const {
  if (windowCount_ == 0) return 0;
  int tmp[WINDOW_SIZE];
  memcpy(tmp, window_, static_cast<size_t>(windowCount_) * sizeof(int));
  for (int i = 1; i < windowCount_; ++i) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = key;
  }
  return tmp[windowCount_ / 2];
}

int SensorFilter::mean() const {
  if (windowCount_ == 0) return 0;
  long sum = 0;
  for (int i = 0; i < windowCount_; ++i) sum += window_[i];
  return static_cast<int>(sum / windowCount_);
}

int SensorFilter::filtered() const {
  if (!initialized_) return 0;
  return (median() + mean()) / 2;
}
