#include "app.hpp"
#include "strip_speed.hpp"
#include <algorithm>
#include "esp_timer.h"

namespace pov {
namespace {
struct SpeedTest {
    bool running = false;
    std::string state = "idle", error;
    uint32_t clockHz = 4000000;
    unsigned duration = 30, pixels = 0, arms = 0, brightness = 0, phase = 0;
    SharedClockProtocol::Protocol protocol = SharedClockProtocol::Protocol::Sk9822;
    int64_t started = 0, elapsed = 0;
    OutputStats stats;
} test;

void finishSpeedTest(const char* state, const std::string& error = {}) {
    test.running = false; test.state = state; test.error = error;
    test.elapsed = test.started ? esp_timer_get_time() - test.started : 0;
    // A fast clock may not have been decoded correctly by the LEDs. Blank at
    // the baseline clock as well before relinquishing ownership of the output.
    const bool restored = output.disableDma() && output.enableDma(4000000);
    output.clear();
    const bool blanked = restored && output.show(0);
    const bool released = output.disableDma();
    if ((!blanked || !released) && test.error.empty())
        test.error = "Could not restore LED output after the test; reboot the controller.";
    if (!test.error.empty()) test.state = "failed";
    runtime.mode = Mode::Stopped; runtime.paused = false;
    runtime.lastActivity = esp_timer_get_time(); runtime.error = test.error;
    ++runtime.generation;
    log("Strip speed test %s: %lu Hz, %llu transfers; %s", test.state.c_str(),
        static_cast<unsigned long>(test.clockHz), static_cast<unsigned long long>(test.stats.count), test.error.c_str());
}
}

bool stripSpeedTestRunning() { return test.running; }
bool startStripSpeedTest(uint32_t clockHz, unsigned seconds, std::string& error) {
    if (!validStripClock(clockHz) || (seconds != 15 && seconds != 30 && seconds != 60 && seconds != 120)) {
        error = "Choose a listed clock rate and test duration"; return false;
    }
    if (!output.ready() || !config.brightness) {
        error = !output.ready() ? "LED output unavailable" : "Set brightness above 0% before testing"; return false;
    }
    stopPlayback(); sequence.close();
    test = {};
    test.clockHz = clockHz; test.duration = seconds;
    test.pixels = config.pixels; test.arms = config.arms;
    test.protocol = static_cast<SharedClockProtocol::Protocol>(config.ledProtocol);
    test.brightness = std::min<unsigned>(10, config.brightness);
    if (!output.enableDma(clockHz) || !output.show(0)) {
        error = "Cannot start strip speed test: " + std::string(esp_err_to_name(output.lastError()));
        finishSpeedTest("failed", error); return false;
    }
    output.stats = {}; // Discard first-transfer peripheral configuration.
    test.running = true; test.state = "running"; test.started = esp_timer_get_time();
    runtime.mode = Mode::StripSpeed; runtime.error.clear(); ++runtime.generation;
    notifyDisplay(); return true;
}
void cancelStripSpeedTest() { if (test.running) finishSpeedTest("cancelled"); }
void stepStripSpeedTest() {
    if (!test.running) return;
    test.elapsed = esp_timer_get_time() - test.started;
    if (test.elapsed >= int64_t(test.duration) * 1000000) { finishSpeedTest("complete"); return; }
    const uint64_t elapsedMs = test.elapsed / 1000;
    test.phase = speedPatternPhase(elapsedMs);
    test.brightness = std::min<unsigned>(10, config.brightness);
    output.clear();
    for (unsigned arm = 0; arm < test.arms; ++arm) {
        for (unsigned pixel = 0; pixel < test.pixels; ++pixel) {
            const auto rgb = speedPatternColor(elapsedMs, pixel, arm, test.pixels);
            // Uniform intensity exposes errors even at the hub. Saved radial
            // dimming remains unchanged for playback and the other tests.
            output.pixel(arm, pixel, rgb[0], rgb[1], rgb[2], false);
        }
    }
    if (!output.show(test.brightness * 255 / 100)) {
        finishSpeedTest("failed", esp_err_to_name(output.lastError())); return;
    }
    test.stats = output.stats;
}
cJSON* stripSpeedTestJson() {
    auto* j = cJSON_CreateObject();
    const int64_t elapsed = test.running ? esp_timer_get_time() - test.started : test.elapsed;
    cJSON_AddBoolToObject(j, "running", test.running);
    cJSON_AddStringToObject(j, "runId", std::to_string(test.started).c_str());
    cJSON_AddStringToObject(j, "state", test.state.c_str());
    cJSON_AddStringToObject(j, "error", test.error.c_str());
    cJSON_AddNumberToObject(j, "clockHz", test.clockHz);
    cJSON_AddNumberToObject(j, "durationSeconds", test.duration);
    cJSON_AddNumberToObject(j, "elapsedSeconds", elapsed / 1000000.0);
    cJSON_AddNumberToObject(j, "pixelsPerStrip", test.pixels);
    cJSON_AddNumberToObject(j, "arms", test.arms);
    cJSON_AddNumberToObject(j, "brightnessPercent", test.brightness);
    cJSON_AddStringToObject(j, "protocol", test.protocol == SharedClockProtocol::Protocol::Apa102 ? "APA102" : "SK9822");
    cJSON_AddStringToObject(j, "pattern", speedPatternName(test.phase));
    cJSON_AddNumberToObject(j, "transmissions", test.stats.count);
    cJSON_AddNumberToObject(j, "updatesPerSecond", elapsed > 0 ? test.stats.count * 1000000.0 / elapsed : 0);
    cJSON_AddNumberToObject(j, "meanTransfer_us", test.stats.count ? double(test.stats.totalUs) / test.stats.count : 0);
    cJSON_AddNumberToObject(j, "maxTransfer_us", test.stats.maxUs);
    cJSON_AddNumberToObject(j, "meanPack_us", test.stats.count ? double(test.stats.totalPackUs) / test.stats.count : 0);
    cJSON_AddNumberToObject(j, "meanSubmitToDone_us", test.stats.count ? double(test.stats.totalCompletionUs) / test.stats.count : 0);
    cJSON_AddNumberToObject(j, "wireTime_us", test.pixels ?
        SharedClockProtocol::frameBytes(test.pixels, test.protocol) * 8.0 * 1000000 / test.clockHz : 0);
    return j;
}
}
