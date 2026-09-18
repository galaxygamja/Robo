#pragma once
#include <stddef.h>

// Supplement a real JSON parser: reject duplicate root keys after decoding,
// embedded NUL bytes, or anything following the root object except whitespace.
inline bool envelopeShape(const char* bytes, size_t length, size_t members) {
  int depth = 0;
  bool inString = false, escaped = false, started = false, finished = false;
  size_t keyCount = 0;
  for (size_t i = 0; i < length; ++i) {
    const char c = bytes[i];
    if (c == 0) return false;
    if (finished) { if (c != ' ' && c != '\r' && c != '\n' && c != '\t') return false; continue; }
    if (inString) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') inString = false;
      continue;
    }
    if (!started && c != ' ' && c != '\r' && c != '\n' && c != '\t') {
      if (c != '{') return false;
      started = true;
    }
    if (c == '"') inString = true;
    else if (c == '{' || c == '[') ++depth;
    else if (c == '}' || c == ']') { --depth; if (depth == 0) finished = true; }
    else if (c == ':' && depth == 1) ++keyCount;
    if (depth < 0) return false;
  }
  return finished && !inString && keyCount == members;
}
