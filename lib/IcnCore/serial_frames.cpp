#include "serial_frames.h"

#include <cstring>

namespace SerialFrames {

namespace {

constexpr char LIMIT_PREFIX[] = "*IL00";

}  // namespace

// ------------------------------------------------------------ LineAssembler

bool LineAssembler::feed(uint8_t c) {
  if (ready_) clear();  // caller forgot clear(): never append to a handled line

  if (c == LF) return false;
  if (c == CR) {
    if (overflowed_ || length_ == 0) {
      clear();
      return false;
    }
    ready_ = true;
    return true;
  }
  if (overflowed_) return false;  // drop the rest of an overlong line
  if (length_ >= CAPACITY) {
    overflowed_ = true;
    ++overflows_;
    return false;
  }
  buffer_[length_++] = static_cast<char>(c);
  buffer_[length_] = '\0';
  return false;
}

void LineAssembler::clear() {
  length_ = 0;
  buffer_[0] = '\0';
  overflowed_ = false;
  ready_ = false;
}

// ---------------------------------------------------------------- HmiParser

bool HmiParser::inLimitPayload() const {
  return length_ >= LIMIT_PREFIX_LEN && memcmp(buffer_, LIMIT_PREFIX, LIMIT_PREFIX_LEN) == 0 &&
         length_ < LIMIT_PREFIX_LEN + LIMIT_PAYLOAD;
}

bool HmiParser::isLimitFrame() const {
  return length_ == LIMIT_PREFIX_LEN + LIMIT_PAYLOAD &&
         memcmp(buffer_, LIMIT_PREFIX, LIMIT_PREFIX_LEN) == 0;
}

const uint8_t *HmiParser::limitPayload() const {
  return reinterpret_cast<const uint8_t *>(buffer_ + LIMIT_PREFIX_LEN);
}

bool HmiParser::feed(uint8_t c) {
  if (ready_) clear();

  if (inLimitPayload()) {
    // Binary section: every byte is data, including CR/LF/0xFF/0x00.
    buffer_[length_++] = static_cast<char>(c);
    buffer_[length_] = '\0';
    if (isLimitFrame()) {
      ready_ = true;
      return true;
    }
    return false;
  }

  if (c == LF) return false;
  if (c == NEXTION_END) {
    clear();
    return false;
  }
  if (c == CR) {
    if (overflowed_ || length_ == 0) {
      clear();
      return false;
    }
    ready_ = true;
    return true;
  }
  if (overflowed_) return false;
  if (length_ >= CAPACITY) {
    overflowed_ = true;
    ++overflows_;
    return false;
  }
  buffer_[length_++] = static_cast<char>(c);
  buffer_[length_] = '\0';
  return false;
}

void HmiParser::clear() {
  length_ = 0;
  buffer_[0] = '\0';
  overflowed_ = false;
  ready_ = false;
}

// ------------------------------------------------------------------- limits

bool limitInRange(int32_t value) { return value >= LIMIT_MIN && value <= LIMIT_MAX; }

bool decodeLimits(const uint8_t *payload, int32_t out[HmiParser::LIMIT_CHANNELS]) {
  int32_t decoded[HmiParser::LIMIT_CHANNELS];
  for (size_t ch = 0; ch < HmiParser::LIMIT_CHANNELS; ++ch) {
    const uint8_t *b = payload + ch * 4;
    uint32_t raw = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
                   (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    decoded[ch] = static_cast<int32_t>(raw);
    if (!limitInRange(decoded[ch])) return false;
  }
  memcpy(out, decoded, sizeof(decoded));
  return true;
}

// ---------------------------------------------------------------------- LFS

bool parseLfs(const char *line, size_t length, LfsReading &out) {
  if (length < 21) return false;
  if (memcmp(line, "#ID:", 4) != 0) return false;
  if (line[4] < '1' || line[4] > '6') return false;

  static const size_t DIGIT_POS[5] = {15, 16, 18, 19, 20};
  LfsReading reading = {};
  reading.id = line[4] - '0';
  bool anyDigit = false;
  for (size_t i = 0; i < 5; ++i) {
    char c = line[DIGIT_POS[i]];
    if (c >= '0' && c <= '9') {
      anyDigit = true;
    } else if (c == ' ') {
      c = '0';
    } else {
      return false;
    }
    reading.hmi[i] = c;
  }
  if (!anyDigit) return false;
  reading.hmi[5] = '\0';
  out = reading;
  return true;
}

}  // namespace SerialFrames
