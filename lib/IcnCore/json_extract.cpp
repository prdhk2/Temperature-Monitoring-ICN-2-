#include "json_extract.h"

#include <cstdio>
#include <cstring>

namespace JsonExtract {

namespace {

// Points just past the ':' that follows "key", skipping whitespace, or
// returns nullptr if the key is absent.
const char *valueStart(const char *json, const char *key) {
  char pattern[32];
  int written = snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  if (written < 0 || static_cast<size_t>(written) >= sizeof(pattern)) return nullptr;

  const char *keyPos = strstr(json, pattern);
  if (!keyPos) return nullptr;
  const char *p = keyPos + written;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p != ':') return nullptr;
  ++p;
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

}  // namespace

bool string(const char *json, const char *key, char *out, size_t outSize) {
  if (outSize == 0) return false;
  const char *p = valueStart(json, key);
  if (!p || *p != '"') return false;
  ++p;

  size_t len = 0;
  while (*p != '"') {
    if (*p == '\0') return false;  // unterminated (e.g. a truncated payload)
    char c = *p++;
    if (c == '\\') {
      switch (*p++) {
        case '"': c = '"'; break;
        case '\\': c = '\\'; break;
        case '/': c = '/'; break;
        case 'n': c = '\n'; break;
        case 't': c = '\t'; break;
        case 'r': c = '\r'; break;
        case 'b': c = '\b'; break;
        case 'f': c = '\f'; break;
        default: return false;  // \uXXXX, a dangling '\' or garbage: never produced for our fields
      }
    }
    if (len + 1 >= outSize) return false;  // would truncate
    out[len++] = c;
  }
  out[len] = '\0';
  return true;
}

bool uint(const char *json, const char *key, uint32_t &out) {
  const char *p = valueStart(json, key);
  if (!p || *p < '0' || *p > '9') return false;

  uint64_t value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10u + static_cast<uint64_t>(*p - '0');
    if (value > UINT32_MAX) return false;
    ++p;
  }
  // "1.5" or "1e3" must not silently read back as 1.
  if (*p == '.' || *p == 'e' || *p == 'E') return false;
  out = static_cast<uint32_t>(value);
  return true;
}

bool hasKey(const char *json, const char *key) { return valueStart(json, key) != nullptr; }

}  // namespace JsonExtract
