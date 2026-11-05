#pragma once

#include <cstdint>

// Noise filter for one thermocouple channel, unchanged from the v4.4 sketch:
// readings outside the physical range of a type-K thermocouple are ignored,
// a jump of more than SPIKE_THRESHOLD from the last accepted sample is
// treated as a spike up to SPIKE_MAX_CONSECUTIVE times in a row (a real
// change is accepted on the next sample after that), and the output is the
// average of the median and the mean of the last WINDOW_SIZE samples.
//
// Pure C++, unit-tested natively in test/test_sensor_filter/.
class SensorFilter {
 public:
  static constexpr int WINDOW_SIZE = 10;
  static constexpr int SPIKE_THRESHOLD = 15;
  static constexpr int SPIKE_MAX_CONSECUTIVE = 3;
  static constexpr int MIN_VALID_TEMP = -50;
  static constexpr int MAX_VALID_TEMP = 1370;

  void begin();

  // Feeds one raw reading. `out` always receives the value to display/use.
  // Returns false when the sample was rejected (out of range or spike).
  bool addSample(int raw, int &out);

  int filtered() const;
  bool initialized() const { return initialized_; }

 private:
  int median() const;
  int mean() const;

  int window_[WINDOW_SIZE] = {0};
  int windowIndex_ = 0;
  int windowCount_ = 0;
  int lastAccepted_ = 0;
  int spikeCount_ = 0;
  bool initialized_ = false;
};
