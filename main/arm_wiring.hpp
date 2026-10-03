#pragma once
#include <cstdint>

namespace pov {
// Both directions are observed from the same viewing side. Playback advances
// with rotation, so an arm ahead in that direction has a positive phase.
inline int armAngleDegrees(unsigned arm, bool armClockwise, bool rotationClockwise) {
    return static_cast<int>(arm) * (armClockwise == rotationClockwise ? 90 : -90);
}
constexpr int64_t ArmOrderTestDurationUs = 60000000;
inline int armOrderTestArm(int64_t elapsedUs, unsigned arms) {
    if (elapsedUs < 0 || elapsedUs >= ArmOrderTestDurationUs || arms < 3 || arms > 4) return -1;
    constexpr int64_t slotUs = 1200000;
    // One second on, then a short all-dark gap before the next connector.
    if (elapsedUs % slotUs >= 1000000) return -1;
    return static_cast<int>((elapsedUs / slotUs) % arms);
}
}
