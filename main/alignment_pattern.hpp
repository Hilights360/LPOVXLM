#pragma once
#include <array>
#include <cstdint>

namespace pov {
struct AlignmentSegment {
    unsigned index;
    double start, end;
    std::array<uint8_t, 3> color;
};

// A solid semicircle and three 9-degree rays centered at 225, 270 and 315
// degrees. Use continuous angular windows so image spoke count, duty and
// strobe cannot break up the green half or hide the alignment reference.
// The caller supplies a wrapped angle in [0, 360).
inline AlignmentSegment alignmentSegment(double angle) {
    constexpr double boundaries[] = {0, 180, 220.5, 229.5, 265.5, 274.5, 310.5, 319.5, 360};
    unsigned index = 0;
    while (index < 7 && angle >= boundaries[index + 1]) ++index;
    const std::array<uint8_t, 3> color = index == 0 ? std::array<uint8_t, 3>{0, 255, 0}
        : index % 2 == 0 ? std::array<uint8_t, 3>{255, 0, 0} : std::array<uint8_t, 3>{};
    return {index, boundaries[index], boundaries[index + 1], color};
}
}
