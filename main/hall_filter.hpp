#pragma once
#include <cstdint>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define POV_HALL_IRAM IRAM_ATTR
#else
#define POV_HALL_IRAM
#endif

namespace pov {
// One index per revolution. The rotor's stated maximum is 150 RPM; allow
// 20% headroom while rejecting electrical edges that cannot be a full turn.
constexpr unsigned MaximumHallRpm = 180;
constexpr int64_t MinimumHallPeriodUs = 60000000 / MaximumHallRpm;
constexpr unsigned HallAverageSamples = 3;

struct HallIndexFilter {
    int64_t last = 0, lastPeriod = 0, lastRejectedInterval = 0;
    int64_t intervals[HallAverageSamples]{}, total = 0;
    uint32_t count = 0, rejected = 0;
    unsigned samples = 0, next = 0;

    bool POV_HALL_IRAM accept(int64_t now) {
        const int64_t elapsed = now - last;
        if (count && elapsed < MinimumHallPeriodUs) {
            ++rejected;
            lastRejectedInterval = elapsed;
            return false; // Preserve both the average and the physical index.
        }
        if (!count || (lastPeriod && elapsed > 1000000 && elapsed > lastPeriod * 3)) {
            // First edge after a stop establishes phase; the next full turn
            // establishes speed. Do not average a stationary gap into RPM.
            samples = next = 0;
            total = lastPeriod = 0;
        } else {
            if (samples == HallAverageSamples) total -= intervals[next];
            else ++samples;
            intervals[next] = elapsed;
            total += elapsed;
            if (++next == HallAverageSamples) next = 0;
            lastPeriod = elapsed;
        }
        last = now;
        ++count;
        return true;
    }

    // Called outside the ISR: keep integer division out of the IRAM path.
    int64_t averagePeriod() const { return samples ? total / samples : 0; }
};
}
#undef POV_HALL_IRAM
