#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace pov {
// This guard is for a wedged bus, not for slow sectors: it stops the lights and
// remounts, which costs far more than a late frame. It therefore sits above the
// host's data-read timeout (SdDataTimeoutMs) and the reader's reopen and retry.
// Prefetching and sweep-boundary presentation hold the last complete image
// through an overrun.
constexpr int64_t FrameLoadTimeoutUs = 1000000;
inline std::string frameLoadTimeoutReason() {
    return "SD frame load exceeded " + std::to_string(FrameLoadTimeoutUs / 1000) + " ms";
}
// User verified clean output at 20 MHz; 26.67 and 40 MHz failed visually.
constexpr uint32_t PlaybackDmaClockHz = 20000000;
constexpr unsigned MaximumPulseColorFrames = 3;
inline bool preparedPulseFits(int64_t now, int64_t deadline, uint32_t frameClocks,
                              unsigned colorFrames = 1) {
    if (!deadline || now >= deadline || !colorFrames || colorFrames > MaximumPulseColorFrames) return false;
    const int64_t wireUs = (uint64_t(frameClocks) * (colorFrames + 1) * 1000000 + PlaybackDmaClockHz - 1) / PlaybackDmaClockHz;
    return deadline - now >= wireUs + 100;
}

// Keep recent peaks rather than a lifetime maximum: a single interrupted
// transfer must not prevent duty from recovering for the rest of a session.
class RecentTransferPeak {
    struct Bucket { int64_t epoch = -1; uint32_t peak = 0; };
    std::array<Bucket, 4> buckets_{};
public:
    void observe(int64_t now, uint32_t duration) {
        const int64_t epoch = now / 500000;
        auto& bucket = buckets_[epoch % buckets_.size()];
        if (bucket.epoch != epoch) bucket = {epoch, 0};
        bucket.peak = std::max(bucket.peak, duration);
    }
    uint32_t peak(int64_t now) const {
        const int64_t epoch = now / 500000;
        uint32_t result = 0;
        for (const auto& bucket : buckets_)
            if (bucket.epoch >= 0 && bucket.epoch <= epoch && epoch - bucket.epoch < 4)
                result = std::max(result, bucket.peak);
        return result;
    }
};

struct DutyCalculation {
    bool available = false, feasible = false, continuous = false;
    unsigned percent = 0;
    double spokeUs = 0;
    uint32_t transferUs = 0, blankUs = 0;
};
inline bool sameSpokeBoundary(double phaseDifference, unsigned spokes) {
    const double position = phaseDifference * spokes / 360;
    return std::abs(position - std::round(position)) < 0.000001;
}
constexpr uint32_t PaintWakeMarginUs = 250;
// The paint budget includes preparation of all arms and LED transmission.
// Reserve time for timer/task wake-up separately, round UP to whole
// percent, and require enough time for a black transfer in the remaining gap.
// 100% is an explicit continuous-output fallback, not a narrow strobe.
inline DutyCalculation calculateDuty(int64_t period, unsigned spokes, uint32_t transferUs, uint32_t blankUs = 0) {
    DutyCalculation result;
    result.transferUs = transferUs;
    result.blankUs = blankUs ? blankUs : transferUs;
    if (period <= 0 || !spokes || !transferUs) return result;
    result.available = true;
    result.spokeUs = double(period) / spokes;
    const double paintUs = transferUs + PaintWakeMarginUs;
    if (paintUs > result.spokeUs) return result;
    const unsigned minimum = std::max(1U, static_cast<unsigned>(std::ceil(paintUs * 100 / result.spokeUs)));
    result.feasible = true;
    if (minimum < 100 && (100 - minimum) * result.spokeUs / 100 >= result.blankUs)
        result.percent = minimum;
    else { result.percent = 100; result.continuous = true; }
    return result;
}
// Preparation already performed is part of the original paint reservation.
// Requiring the entire reservation again before each arm rejected otherwise
// valid paints, especially at the minimum automatically calculated duty.
inline bool remainingPaintFits(int64_t started, int64_t now, int64_t deadline,
                               uint32_t paintBudget, uint32_t submitBudget) {
    return deadline > 0 && std::max(started + paintBudget, now + submitBudget) <= deadline;
}
inline bool frameLoadExpired(int64_t started, int64_t now) {
    return started > 0 && now - started >= FrameLoadTimeoutUs;
}
struct PlaybackAdvance { uint32_t frame; bool finished; };
inline PlaybackAdvance advancePlayback(uint32_t current, uint64_t advance, uint32_t count, bool loop) {
    if (!count) return {0, true};
    const uint32_t last = count - 1;
    // Even after a late wake, show the last frame before stopping on the next tick.
    if (!loop && current >= last && advance) return {last, true};
    const uint64_t next = uint64_t(current) + advance;
    return {loop ? static_cast<uint32_t>(next % count)
                 : static_cast<uint32_t>(std::min<uint64_t>(next, last)), false};
}
// Four equally spaced arms cover the image in a quarter turn. Other active
// arm counts, non-quarter image widths, or unequal phase corrections use a
// full turn so a frame is never replaced before the image can be completed.
inline unsigned playbackSweepsPerTurn(unsigned arms, unsigned spokes, const std::array<float, 4>& phases) {
    if (arms != 4 || !spokes || spokes % 4) return 1;
    for (unsigned arm = 1; arm < arms; ++arm)
        if (std::abs(std::remainder(double(phases[arm]) - phases[0], 360.0)) > 0.000001) return 1;
    return 4;
}
class PlaybackSweep {
    bool initialized_ = false;
    uint32_t hallCount_ = 0;
    unsigned sweeps_ = 0;
    double phase_ = 0;
    int64_t sweep_ = 0;
public:
    void reset() { initialized_ = false; }
    bool boundary(uint32_t hallCount, int64_t elapsed, int64_t period, unsigned sweeps, double phase) {
        if (period <= 0 || !sweeps) { reset(); return true; }
        phase = std::fmod(phase, 360.0);
        if (phase < 0) phase += 360.0;
        const int64_t sweep = int64_t(hallCount) * sweeps + static_cast<int64_t>(std::floor(
            (double(std::max<int64_t>(0, elapsed)) / period + phase / 360.0) * sweeps));
        // A real Hall edge may arrive just after the predicted wrap. Do not
        // latch twice there, or when a period correction moves phase backward.
        const bool fresh = !initialized_ || hallCount < hallCount_ || sweeps != sweeps_ || phase != phase_;
        const bool changed = !fresh && sweep > sweep_;
        if (fresh || changed) sweep_ = sweep;
        initialized_ = true;
        hallCount_ = hallCount; sweeps_ = sweeps; phase_ = phase;
        return changed;
    }
};
// Fractional image-spoke position, shared by sequence playback and LED tests.
// Phase includes the physical connector angle and its calibration offset.
inline double spokePosition(int64_t elapsed, int64_t period, unsigned spokes, double phase) {
    if (period <= 0 || !spokes) return 0;
    double angle = std::fmod(elapsed * 360.0 / period + phase, 360.0);
    if (angle < 0) angle += 360.0;
    return angle * spokes / 360.0;
}
struct SpokeGate {
    bool inside = false, fits = false, canStart = false;
    double closeInUs = 0, nextInUs = 0;
};
inline SpokeGate spokeGate(double fraction, double spokeUs, unsigned duty,
                           bool strobe, double widthDegrees, unsigned spokes, double transferUs, double blankUs = 0) {
    double start = 0, end = duty / 100.0;
    if (strobe) {
        const double half = std::min(0.5, widthDegrees * spokes / 720.0);
        start = 0.5 - half; end = 0.5 + half;
    }
    SpokeGate result;
    result.inside = duty && fraction >= start && fraction < end;
    // Budget both painting and the following black transfer. Duty=100 has
    // no separate black interval. Impossible windows stay dark, not smeared.
    const double hold = (end - start) * spokeUs;
    const double gap = (1 - end + start) * spokeUs;
    result.fits = duty && hold >= transferUs && (gap < 0.001 || gap >= (blankUs > 0 ? blankUs : transferUs));
    result.closeInUs = (end - fraction) * spokeUs;
    result.canStart = result.inside && result.fits && result.closeInUs >= transferUs;
    const double next = fraction < start ? start : fraction < end ? end : 1.0;
    result.nextInUs = std::max(50.0, (next - fraction) * spokeUs);
    return result;
}
}
