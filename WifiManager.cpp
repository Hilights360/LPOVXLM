#include "WifiManager.h"

#include <WiFi.h>
#include <esp_system.h>

String defaultStationId() {
  char buf[16];
  uint64_t mac = ESP.getEfuseMac();
  uint32_t suffix = (uint32_t)(mac & 0xFFFFFFu);
  snprintf(buf, sizeof(buf), "pov-%06X", (unsigned int)suffix);
  return String(buf);
}

void applyStationHostname() {
  if (!g_stationId.length()) g_stationId = defaultStationId();
  WiFi.setHostname(g_stationId.c_str());
  WiFi.softAPsetHostname(g_stationId.c_str());
}

void markStationState(bool connected) {
  if (connected != g_staConnected) {
    g_staConnected = connected;
    Serial.printf("[WIFI] Station %s\n", connected ? "connected" : "disconnected");
  }
  if (connected) g_staConnecting = false;
}

void connectWifiStation() {
  if (!g_staSsid.length()) {
    markStationState(false);
    return;
  }
  applyStationHostname();
  Serial.printf("[WIFI] Connecting to SSID '%s'...\n", g_staSsid.c_str());
  WiFi.disconnect(false, true);
  markStationState(false);
  WiFi.begin(g_staSsid.c_str(), g_staPass.c_str());
  g_staConnecting = true;
  g_staConnectStartMs = millis();
}

void pollWifiStation() {
  if (!g_staSsid.length()) return;
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    markStationState(true);
  } else {
    if (g_staConnected) markStationState(false);
    if (g_staConnecting && millis() - g_staConnectStartMs > 20000) {
      Serial.println("[WIFI] Retry station connection");
      connectWifiStation();
    }
  }
}
