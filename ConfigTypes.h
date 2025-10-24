#pragma once
#include <Arduino.h>

// Shared constants for spinner geometry
constexpr uint8_t  MAX_ARMS               = 4;
constexpr uint16_t DEFAULT_PIXELS_PER_ARM = 144;
constexpr uint16_t MAX_PIXELS_PER_ARM     = 1024;

// Utility clamps reused across modules
inline uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline uint8_t clampArmCount(int32_t v) {
  if (v < 1) return 1;
  if (v > MAX_ARMS) return MAX_ARMS;
  return static_cast<uint8_t>(v);
}

inline uint16_t clampPixelsPerArm(int32_t v) {
  if (v < 1) return 1;
  if (v > MAX_PIXELS_PER_ARM) return MAX_PIXELS_PER_ARM;
  return static_cast<uint16_t>(v);
}

enum SdBusPreference : uint8_t { SD_BUS_AUTO=0, SD_BUS_1BIT=1, SD_BUS_4BIT=4 };

// Presence flags for what exists in NVS (preferences)
struct PrefPresence {
  bool brightness=false;
  bool fps=false;
  bool startCh=false;
  bool spokes=false;
  bool arms=false;
  bool pixels=false;
  bool staSsid=false;
  bool staPass=false;
  bool station=false;
  bool sdMode=false;
  bool sdFreq=false;
  bool autoplay=false;
  bool watchdog=false;
  bool bgEffectEnable=false;
  bool bgEffectPath=false;
  bool outMode=false;     // presence flag ONLY (no value here)
};

// Values loaded from /config/settings.ini (SD backup)
struct SettingsData {
  bool hasBrightness=false; uint8_t  brightness=0;
  bool hasFps=false;        uint16_t fps=0;
  bool hasStartCh=false;    uint32_t startCh=0;
  bool hasSpokes=false;     uint16_t spokes=0;
  bool hasArms=false;       uint8_t  arms=0;
  bool hasPixels=false;     uint16_t pixels=0;
  bool hasStaSsid=false;    String   staSsid;
  bool hasStaPass=false;    String   staPass;
  bool hasStation=false;    String   stationId;
  bool hasSdMode=false;     uint8_t  sdMode=0;
  bool hasSdFreq=false;     uint32_t sdFreq=0;
  bool hasAutoplay=false;   bool     autoplay=false;
  bool hasWatchdog=false;   bool     watchdog=false;
  bool hasBgEffectEnable=false; bool bgEffectEnable=false;
  bool hasBgEffectPath=false;   String bgEffectPath;
  bool hasOutMode=false;    uint8_t  outMode=0;   // 0=SPI, 1=Parallel
};
