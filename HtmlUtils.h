#pragma once

#include <Arduino.h>

extern const char* const BG_EFFECTS_DIR;

bool isFseqName(const String& name);
bool isBgEffectPath(const String& path);
String sanitizeBgEffectPath(const String& in);
String bgEffectDisplayName(const String& path);
String htmlEscape(const String& in);
String urlEncode(const String& in);
String dirnameOf(const String& path);
String joinPath(const String& dir, const String& name);
