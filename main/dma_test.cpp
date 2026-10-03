#include "app.hpp"
#include <algorithm>
#include "esp_timer.h"

namespace pov {
namespace {
struct Benchmark {
    bool running = false;
    std::string state = "idle", phase = "gpio", error;
    uint32_t clockHz = 4000000;
    unsigned pixels = 0, arms = 0, brightness = 0, frames = 0;
    SharedClockProtocol::Protocol protocol = SharedClockProtocol::Protocol::Sk9822;
    int64_t nextAt = 0;
    OutputStats gpio, dma;
} test;
constexpr unsigned BaselineFrames = 10, DmaFrames = 100;

void finish(const char* state, const std::string& error = {}) {
    test.running = false; test.state = state; test.error = error;
    output.clear();
    if (output.ready() && !output.show(0) && test.error.empty()) {
        test.state = "failed"; test.error = esp_err_to_name(output.lastError());
    }
    if (!output.disableDma()) {
        test.state = "failed";
        test.error = "DMA did not finish; clock disconnected. Reboot the controller before another test.";
    }
    runtime.mode = Mode::Stopped; runtime.paused = false;
    runtime.lastActivity = esp_timer_get_time();
    runtime.error = test.error;
    ++runtime.generation;
    log("DMA test %s: %u MHz, %u pixels, GPIO mean %.1f us, DMA mean %.1f us; %s",
        test.state.c_str(), unsigned(test.clockHz / 1000000), test.pixels,
        test.gpio.count ? double(test.gpio.totalUs) / test.gpio.count : 0,
        test.dma.count ? double(test.dma.totalUs) / test.dma.count : 0, test.error.c_str());
}
cJSON* timingJson(const OutputStats& stats) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddNumberToObject(j, "transmissions", stats.count);
    cJSON_AddNumberToObject(j, "minTotal_us", stats.minUs);
    cJSON_AddNumberToObject(j, "maxTotal_us", stats.maxUs);
    cJSON_AddNumberToObject(j, "meanTotal_us", stats.count ? double(stats.totalUs) / stats.count : 0);
    cJSON_AddNumberToObject(j, "meanPack_us", stats.count ? double(stats.totalPackUs) / stats.count : 0);
    cJSON_AddNumberToObject(j, "meanSubmitToDone_us", stats.count ? double(stats.totalCompletionUs) / stats.count : 0);
    cJSON_AddNumberToObject(j, "maxSubmitToDone_us", stats.maxCompletionUs);
    return j;
}
}
bool dmaTestRunning() { return test.running; }
bool startDmaTest(uint32_t clockHz, std::string& error) {
    if (test.running) { error = "A DMA test is already running"; return false; }
    if (!output.ready()) { error = "LED output unavailable; reboot after a DMA timeout"; return false; }
    if (clockHz != 4000000 && clockHz != 8000000 && clockHz != 16000000) {
        error = "Choose 4, 8, or 16 MHz"; return false;
    }
    stopPlayback();
    test = {};
    test.clockHz = clockHz; test.pixels = config.pixels; test.arms = config.arms;
    test.protocol = static_cast<SharedClockProtocol::Protocol>(config.ledProtocol);
    test.brightness = std::min<unsigned>(10, config.brightness);
    test.state = "running"; test.running = true;
    runtime.error.clear(); runtime.mode = Mode::DmaTest;
    output.stats = {};
    ++runtime.generation; notifyDisplay();
    return true;
}
void cancelDmaTest() { if (test.running) finish("cancelled"); }
void stepDmaTest() {
    if (!test.running || esp_timer_get_time() < test.nextAt) return;
    if (test.phase == "gpio" && test.frames == BaselineFrames) {
        if (!output.enableDma(test.clockHz)) { finish("failed", esp_err_to_name(output.lastError())); return; }
        output.clear();
        // Discard the first transfer, which also configures the LCD device.
        if (!output.show(0)) { finish("failed", esp_err_to_name(output.lastError())); return; }
        output.stats = {}; test.phase = "dma"; test.frames = 0;
    }
    output.clear();
    if (!(test.frames & 1)) {
        for (unsigned arm = 0; arm < test.arms; ++arm) {
            for (unsigned pixel = 0; pixel < test.pixels; ++pixel) {
                // A moving white marker also exercises changing pixel data.
                const bool marker = pixel == (test.frames / 2) % test.pixels;
                output.pixel(arm, pixel, marker || arm == 0 || arm == 3 ? 255 : 0,
                    marker || arm == 1 || arm == 3 ? 255 : 0, marker || arm == 2 || arm == 3 ? 255 : 0);
            }
        }
    }
    if (!output.show(test.brightness * 255 / 100)) { finish("failed", esp_err_to_name(output.lastError())); return; }
    ++test.frames;
    if (test.phase == "gpio") test.gpio = output.stats;
    else test.dma = output.stats;
    // Hall interrupts may also wake the display task; retain the test cadence.
    test.nextAt = esp_timer_get_time() + 50000;
    if (test.phase == "dma" && test.frames == DmaFrames) finish("complete");
}
cJSON* dmaTestJson() {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "running", test.running);
    cJSON_AddStringToObject(j, "state", test.state.c_str());
    cJSON_AddStringToObject(j, "phase", test.phase.c_str());
    cJSON_AddStringToObject(j, "error", test.error.c_str());
    cJSON_AddNumberToObject(j, "clockHz", test.clockHz);
    cJSON_AddNumberToObject(j, "pixelsPerStrip", test.pixels);
    cJSON_AddNumberToObject(j, "arms", test.arms);
    cJSON_AddNumberToObject(j, "brightnessPercent", test.brightness);
    cJSON_AddNumberToObject(j, "framesCompleted", test.frames);
    cJSON_AddNumberToObject(j, "framesTarget", test.phase == "gpio" ? BaselineFrames : DmaFrames);
    const unsigned bits = test.pixels ? SharedClockProtocol::frameBytes(test.pixels, test.protocol) * 8 : 0;
    cJSON_AddStringToObject(j, "protocol", test.protocol == SharedClockProtocol::Protocol::Apa102 ? "APA102" : "SK9822");
    cJSON_AddNumberToObject(j, "clocksPerStrip", bits);
    cJSON_AddNumberToObject(j, "theoreticalWire_us", double(bits) * 1000000 / test.clockHz);
    cJSON_AddItemToObject(j, "gpio", timingJson(test.gpio));
    cJSON_AddItemToObject(j, "dma", timingJson(test.dma));
    cJSON_AddNumberToObject(j, "speedup", test.gpio.count && test.dma.count && test.dma.totalUs ?
        (double(test.gpio.totalUs) / test.gpio.count) / (double(test.dma.totalUs) / test.dma.count) : 0);
    return j;
}
}
