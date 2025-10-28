#include "DebugLog.h"

#include <stdarg.h>
#include <stdio.h>

namespace {

constexpr size_t kMaxLines = 200;
constexpr size_t kMaxPrintf = 384;

bool g_serialEnabled = false;
String g_lines[kMaxLines];
size_t g_lineCount = 0;
size_t g_nextIndex = 0;

void storeLine(const String &line) {
  g_lines[g_nextIndex] = line;
  g_nextIndex = (g_nextIndex + 1) % kMaxLines;
  if (g_lineCount < kMaxLines) {
    ++g_lineCount;
  }
}

void appendSplit(const String &msg) {
  if (!msg.length()) {
    storeLine(String());
    return;
  }
  int start = 0;
  const int len = msg.length();
  while (start < len) {
    int end = msg.indexOf('\n', start);
    if (end < 0) end = len;
    String piece = msg.substring(start, end);
    while (piece.length() && (piece.endsWith("\r"))) {
      piece.remove(piece.length() - 1);
    }
    storeLine(piece);
    if (end >= len) break;
    start = end + 1;
  }
}

void appendMessage(const String &msg) {
  appendSplit(msg);
}

String collectLines(const char *separator) {
  String out;
  if (!separator) separator = "\n";
  size_t count = g_lineCount;
  if (count == 0) return out;
  out.reserve(count * 40);
  size_t start = (g_lineCount == kMaxLines) ? g_nextIndex : 0;
  for (size_t i = 0; i < count; ++i) {
    size_t idx = (start + i) % kMaxLines;
    out += g_lines[idx];
    if (i + 1 < count) out += separator;
  }
  return out;
}

}  // namespace

namespace DebugLog {

void begin(bool serialEnabled) {
  g_serialEnabled = serialEnabled;
  if (g_serialEnabled) {
    Serial.begin(115200);
    delay(800);
  }
}

void setSerialEnabled(bool enabled) {
  if (enabled == g_serialEnabled) return;
  g_serialEnabled = enabled;
  if (g_serialEnabled) {
    Serial.begin(115200);
    delay(800);
  }
}

bool serialEnabled() {
  return g_serialEnabled;
}

void print(const String &msg) {
  appendMessage(msg);
  if (g_serialEnabled) Serial.print(msg);
}

void print(const char *msg) {
  if (!msg) return;
  appendMessage(String(msg));
  if (g_serialEnabled) Serial.print(msg);
}

void print(const __FlashStringHelper *msg) {
  if (!msg) return;
  String s(msg);
  appendMessage(s);
  if (g_serialEnabled) Serial.print(msg);
}

void println(const String &msg) {
  appendMessage(msg + "\n");
  if (g_serialEnabled) Serial.println(msg);
}

void println(const char *msg) {
  if (!msg) return;
  appendMessage(String(msg) + "\n");
  if (g_serialEnabled) Serial.println(msg);
}

void println(const __FlashStringHelper *msg) {
  if (!msg) return;
  String s(msg);
  appendMessage(s + "\n");
  if (g_serialEnabled) Serial.println(msg);
}

void printf(const char *fmt, ...) {
  if (!fmt) return;
  char buf[kMaxPrintf];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  appendMessage(String(buf));
  if (g_serialEnabled) Serial.printf("%s", buf);
}

String logsText() {
  return collectLines("\n");
}

String logsHtmlPre() {
  String text = logsText();
  String html;
  html.reserve(text.length() + 64);
  for (size_t i = 0; i < text.length(); ++i) {
    char c = text[i];
    switch (c) {
      case '&': html += "&amp;"; break;
      case '<': html += "&lt;"; break;
      case '>': html += "&gt;"; break;
      default: html += c; break;
    }
  }
  return html;
}

void clear() {
  for (size_t i = 0; i < kMaxLines; ++i) {
    g_lines[i] = String();
  }
  g_lineCount = 0;
  g_nextIndex = 0;
}

}  // namespace DebugLog
