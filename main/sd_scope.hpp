#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include "BoardPins.h"

namespace pov {
constexpr int SdScopeAllPins = -1;
constexpr int SdScopePins[] = {BoardPins::SdD0, BoardPins::SdD1, BoardPins::SdCmd,
                              BoardPins::SdD2, BoardPins::SdD3, BoardPins::SdClk};
struct SdScopeOptions {
    int pin = BoardPins::SdD0;
    unsigned intervalUs = 100; // Requested interval; each sample carries its actual time.
    bool wifiOff = false;
};
constexpr unsigned SdScopeSamples = 1024;
inline const char* sdScopePinName(int pin) {
    if (pin == BoardPins::SdD0) return "DAT0";
    if (pin == BoardPins::SdD1) return "DAT1";
    if (pin == BoardPins::SdD2) return "DAT2";
    if (pin == BoardPins::SdD3) return "DAT3";
    if (pin == BoardPins::SdCmd) return "CMD";
    if (pin == BoardPins::SdClk) return "CLK";
    return nullptr;
}
struct SdScopePoint {
    uint32_t timeUs = 0;
    int raw = -1, millivolts = -1; // -1 is a missed/invalid reading, never a zero-volt sample.
    bool upperLimit = false; // Calibration is not a voltage measurement outside the ADC range.
};
struct SdScopeResult {
    std::vector<SdScopePoint> points;
    bool calibrated = false, cancelled = false;
    unsigned adcUnit = 0;
    int pin = SdScopeAllPins;
    int64_t startedUs = 0;
    std::string error;
    std::string calibrationError, readError;
};
// Caller has exclusive storage ownership and has detached the SD host.
bool captureSdScope(const SdScopeOptions&, std::atomic<bool>& cancel, SdScopeResult&, std::string& error);
// Return every SD pad from the RTC/ADC mux to digital inputs with normal pulls.
bool restoreSdScopePins(std::string& error);
}
