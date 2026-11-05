#pragma once

#include <cstddef>
#include <cstdint>

// Byte-level framing of the three serial links of the ICN-2 controller.
// Pure C++, unit-tested natively in test/test_serial_frames/.
//
// The v4.4 sketch collected each link into an Arduino String with no length
// limit. Three problems this replaces:
//   - a line without its CR (noise, a cable unplugged mid-frame, the LFS
//     answering while the HMI is on the settings page) grew the String until
//     the heap ran out and the controller crashed;
//   - two panel commands arriving in one loop() pass were glued into one
//     string ("*I110*I111") and both were ignored;
//   - the HMI's limit frame "*IL00" carries five raw little-endian int32,
//     so a limit whose bytes contain 0x0A, 0x0D or 0xFF (10, 13, 255, 266,
//     269, 511, 522, 525, 767, ... degrees) was cut short or wiped, and the
//     shortened frame then stored limits of 0 in EEPROM.
namespace SerialFrames {

constexpr uint8_t CR = 0x0D;
constexpr uint8_t LF = 0x0A;
constexpr uint8_t NEXTION_END = 0xFF;

// Generic CR-terminated text line (relay panel, LFS flow meters). LF is
// ignored, an overlong line is discarded as a whole.
class LineAssembler {
 public:
  static constexpr size_t CAPACITY = 64;

  // Returns true when a complete, non-empty line is ready in line()/length().
  // The caller must call clear() after handling it.
  bool feed(uint8_t c);
  void clear();

  const char *line() const { return buffer_; }
  size_t length() const { return length_; }
  uint32_t overflows() const { return overflows_; }

 private:
  char buffer_[CAPACITY + 1] = {0};
  size_t length_ = 0;
  bool overflowed_ = false;
  bool ready_ = false;
  uint32_t overflows_ = 0;
};

// Nextion HMI link. Text commands end with CR; LF is ignored; 0xFF (the
// Nextion return-frame terminator) resets the line. A command starting with
// "*IL00" is followed by LIMIT_PAYLOAD raw bytes taken verbatim, whatever
// their value; the frame is complete as soon as they have arrived (the CR
// that follows then ends an empty line, which is ignored).
class HmiParser {
 public:
  static constexpr size_t CAPACITY = 64;
  static constexpr size_t LIMIT_PREFIX_LEN = 5;  // "*IL00"
  static constexpr size_t LIMIT_CHANNELS = 5;
  static constexpr size_t LIMIT_PAYLOAD = LIMIT_CHANNELS * 4;

  bool feed(uint8_t c);
  void clear();

  const char *command() const { return buffer_; }
  size_t length() const { return length_; }
  bool isLimitFrame() const;
  // Only valid when isLimitFrame(): the LIMIT_PAYLOAD raw bytes.
  const uint8_t *limitPayload() const;
  uint32_t overflows() const { return overflows_; }

 private:
  bool inLimitPayload() const;

  char buffer_[CAPACITY + 1] = {0};
  size_t length_ = 0;
  bool overflowed_ = false;
  bool ready_ = false;
  uint32_t overflows_ = 0;
};

// Decodes the five little-endian int32 limits of an "*IL00" frame. Returns
// false (and leaves out[] untouched) if any of them is outside
// [LIMIT_MIN, LIMIT_MAX] - such a frame is corrupt and must not be saved.
constexpr int32_t LIMIT_MIN = 1;
constexpr int32_t LIMIT_MAX = 1370;  // type-K upper end, same as the filter
bool decodeLimits(const uint8_t *payload, int32_t out[HmiParser::LIMIT_CHANNELS]);
bool limitInRange(int32_t value);

// LFS flow meter answer, e.g. "#ID:3 ... 12.345" with the volume digits at
// fixed offsets 15-16 and 18-20 (as read by the v4.4 sketch). Spaces in the
// digit positions count as '0'. Returns false for anything else (wrong id,
// short line such as a "#ID:3-RESET" echo, non-digits).
struct LfsReading {
  int id;           // 1..6
  char hmi[6];      // "12345" - value for X_VolLFn.val (Nextion xfloat)
};
bool parseLfs(const char *line, size_t length, LfsReading &out);

}  // namespace SerialFrames
