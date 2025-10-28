#pragma once

#include <Arduino.h>

namespace DebugLog {

void begin(bool serialEnabled);
void setSerialEnabled(bool enabled);
bool serialEnabled();

void print(const String &msg);
void print(const char *msg);
void print(const __FlashStringHelper *msg);

void println(const String &msg);
void println(const char *msg);
void println(const __FlashStringHelper *msg);

void printf(const char *fmt, ...);

String logsText();
String logsHtmlPre();
void clear();

}  // namespace DebugLog
