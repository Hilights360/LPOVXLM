// ConfigTypes.h
#pragma once
#include <Arduino.h>

struct PrefPresence {
  bool brightness=false;
  bool fps=false;
  bool startCh=false;
  bool spokes=false;
  bool arms=false;
  bool pixels=false;
  bool stride=false;
  bool staSsid=false;
  bool staPass=false;
  bool station=false;
  bool sdMode=false;
  bool sdFreq=false;
};

struct SettingsData {
  bool hasBrightness=false; uint8_t  brightness=0;
  bool hasFps=false;        uint16_t fps=0;
  bool hasStartCh=false;    uint32_t startCh=0;
  bool hasSpokes=false;     uint16_t spokes=0;
  bool hasArms=false;       uint8_t  arms=0;
  bool hasPixels=false;     uint16_t pixels=0;
  bool hasStride=false;     uint8_t  stride=0;
  bool hasStaSsid=false;    String   staSsid;
  bool hasStaPass=false;    String   staPass;
  bool hasStation=false;    String   stationId;
  bool hasSdMode=false;     uint8_t  sdMode=0;
  bool hasSdFreq=false;     uint32_t sdFreq=0;
};
