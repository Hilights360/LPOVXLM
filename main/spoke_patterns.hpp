#pragma once
#include <array>
#include <cstdint>

namespace pov {
enum class SpokePattern { Quarters, Alternating };

// Colors belong to positions around the disk, not to physical arms or frames.
// Uneven spoke counts split into quarters differing by at most one spoke.
inline std::array<uint8_t, 3> spokeColor(SpokePattern pattern, unsigned spoke, unsigned count) {
    if (!count) return {};
    constexpr std::array<uint8_t, 3> colors[] = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}
    };
    spoke %= count;
    const unsigned index = pattern == SpokePattern::Quarters
        ? static_cast<unsigned>(uint64_t(spoke) * 4 / count) : spoke % 4;
    return colors[index];
}
}
