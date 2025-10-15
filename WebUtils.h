#pragma once

#include <Arduino.h>

String htmlEscape(const String &in);
String urlEncode(const String &in);
String dirnameOf(const String &path);
String joinPath(const String &dir, const String &name);
String baseName(const String &path);

