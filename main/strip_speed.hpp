#pragma once
#include <array>
#include <cstdint>

namespace pov {
// IDF 5.3's PLL160M I80 source is divided by two, then an integer prescaler.
// These rates select known divider steps; arbitrary requests can round upward.
// Round 80 MHz / 3 DOWN to integer Hz when requesting it: rounding up to
// 26,666,667 Hz would make IDF select divisor 2 and output 40 MHz instead.
inline constexpr uint32_t StripClockRates[] = {2000000, 4000000, 8000000, 10000000,
                                               16000000, 20000000, 80000000U / 3, 40000000};
inline bool validStripClock(uint32_t hz) {
    for (auto rate : StripClockRates) if (hz == rate) return true;
    return false;
}
inline unsigned speedPatternPhase(uint64_t elapsedMs) { return static_cast<unsigned>((elapsedMs / 2000) % 6); }
inline const char* speedPatternName(unsigned phase) {
    constexpr const char* names[] = {"All red", "All green", "All blue", "All white",
        "RGBW blocks with moving white marker", "Alternating white and dark pixels"};
    return names[phase % 6];
}
inline std::array<uint8_t, 3> speedPatternColor(uint64_t elapsedMs, unsigned pixel,
                                              unsigned arm, unsigned pixels) {
    constexpr std::array<uint8_t, 3> colors[] = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}
    };
    const auto phase = speedPatternPhase(elapsedMs);
    if (phase < 4) return colors[phase];
    if (phase == 4) {
        if (pixels && pixel == (elapsedMs / 100) % pixels) return colors[3];
        return colors[(pixel / 8 + arm) % 4];
    }
    return (pixel + elapsedMs / 100) % 2 ? colors[3] : std::array<uint8_t, 3>{};
}
}
