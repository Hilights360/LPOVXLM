#pragma once

#include <Arduino.h>

String defaultStationId();
void applyStationHostname();
void markStationState(bool connected);
void connectWifiStation();
void pollWifiStation();

extern String g_staSsid;
extern String g_staPass;
extern String g_stationId;
extern bool g_staConnecting;
extern bool g_staConnected;
extern uint32_t g_staConnectStartMs;
