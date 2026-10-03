#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

namespace pov {
// Radial index zero is the outer tip; the last index is nearest the hub.
// Return a Q8 gain to apply before the existing overall brightness scaling.
// A one-pixel arm has no radial extent and retains regular brightness.
inline uint16_t radialBrightnessGain(unsigned pixel, unsigned pixels, unsigned regular,
                                    unsigned center, bool enabled) {
    if (!enabled) return 256;
    if (!regular) return 0;
    if (pixels <= 1 || center >= regular) return 256;
    const unsigned steps = pixels - 1;
    pixel = std::min(pixel, steps);
    const unsigned denominator = regular * steps;
    const unsigned numerator = denominator - (regular - center) * pixel;
    return static_cast<uint16_t>((numerator * 256U + denominator / 2) / denominator);
}
inline std::array<uint8_t, 3> radialColor(std::array<uint8_t, 3> rgb, unsigned pixel,
                                       unsigned pixels, unsigned regular, unsigned center,
                                       bool enabled, bool inputAtHub = false) {
    // Input is a wire index, after image reversal. This keeps the fade at the
    // hub even when image orientation changes or a strip is wired differently.
    const unsigned radialIndex = inputAtHub && pixels ? pixels - 1 - std::min(pixel, pixels - 1) : pixel;
    const uint16_t gain = radialBrightnessGain(radialIndex, pixels, regular, center, enabled);
    for (auto& component : rgb)
        component = static_cast<uint8_t>((unsigned(component) * gain + 128) >> 8);
    return rgb;
}

// The brightness profile changes only when settings or strip length change.
// Keep its divisions out of each pixel's rendering path.
template<unsigned Capacity> class RadialGainCache {
    std::array<uint16_t, Capacity> gains_{};
    unsigned pixels_ = 0, regular_ = 0, center_ = 0;
public:
    uint16_t gain(unsigned pixel, unsigned pixels, unsigned regular, unsigned center,
                  bool enabled, bool inputAtHub) {
        if (!enabled) return 256;
        if (!regular) return 0;
        if (pixels <= 1 || center >= regular) return 256;
        const unsigned radial = inputAtHub ? pixels - 1 - std::min(pixel, pixels - 1) : pixel;
        if (pixels > Capacity) return radialBrightnessGain(radial, pixels, regular, center, true);
        if (pixels_ != pixels || regular_ != regular || center_ != center) {
            for (unsigned i = 0; i < pixels; ++i)
                gains_[i] = radialBrightnessGain(i, pixels, regular, center, true);
            pixels_ = pixels; regular_ = regular; center_ = center;
        }
        return gains_[std::min(radial, pixels - 1)];
    }
};
}
