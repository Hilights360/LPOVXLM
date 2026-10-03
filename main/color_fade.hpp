#pragma once
#include <array>
#include <cstdint>

namespace pov {
constexpr uint32_t ColorFadeFrameMs = 20;
constexpr uint32_t ColorFadeCycleMs = 12000;

// One shared RGB value for every pixel and arm. Six continuous transitions:
// red -> yellow -> green -> cyan -> blue -> magenta -> red.
inline std::array<uint8_t, 3> colorFadeAt(uint64_t elapsedMs) {
    constexpr uint32_t segmentMs = ColorFadeCycleMs / 6;
    const uint32_t phase = elapsedMs % ColorFadeCycleMs;
    const auto rising = static_cast<uint8_t>((phase % segmentMs) * 255 / segmentMs);
    const auto falling = static_cast<uint8_t>(255 - rising);
    switch (phase / segmentMs) {
    case 0: return {255, rising, 0};
    case 1: return {falling, 255, 0};
    case 2: return {0, 255, rising};
    case 3: return {0, falling, 255};
    case 4: return {rising, 0, 255};
    default: return {255, 0, falling};
    }
}
}
