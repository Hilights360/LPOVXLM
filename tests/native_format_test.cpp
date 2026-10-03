#include "fseq_format.hpp"
#include "SharedClockProtocol.h"
#include "BoardPins.h"
#include "wifi_retry.hpp"
#include "hostname.hpp"
#include "color_fade.hpp"
#include "spoke_patterns.hpp"
#include "playback_timing.hpp"
#include "radial_brightness.hpp"
#include "strip_speed.hpp"
#include "arm_wiring.hpp"
#include "alignment_pattern.hpp"
#include "fseq_settings.hpp"
#include "hall_filter.hpp"
#include <cassert>
#include <iostream>

using namespace pov;
static void radialSequenceDirection() {
    // The older 80-spoke export advances from source pixel 143 to 0; the
    // current 256-spoke export advances from 0 to 143. Both must move outward
    // on the same hub-fed PCB. Check every lane through DMA packing, with
    // brightness shaping remaining at the physical hub in either mapping.
    for (unsigned pixels : {1U, 5U, 144U})
      for (bool tipFirst : {false, true})
        for (auto protocol : {SharedClockProtocol::Protocol::Apa102, SharedClockProtocol::Protocol::Sk9822}) {
            unsigned previousIntensity = 0;
            for (unsigned radius = 0; radius < pixels; ++radius) {
                std::vector<uint8_t> rgb(4 * pixels * 3);
                std::vector<uint8_t> packed(SharedClockProtocol::frameBytes(static_cast<uint16_t>(pixels), protocol) * 8);
                const unsigned imagePixel = tipFirst ? pixels - 1 - radius : radius;
                for (unsigned arm = 0; arm < 4; ++arm) {
                    assert(BoardPins::ArmInputAtHub[arm]);
                    const unsigned wirePixel = BoardPins::imagePixelToWire(arm, imagePixel, pixels, tipFirst);
                    const auto color = radialColor({255, 255, 255}, wirePixel, pixels, 50, 10, true,
                        BoardPins::ArmInputAtHub[arm]);
                    std::copy(color.begin(), color.end(), rgb.begin() + (arm * pixels + wirePixel) * 3);
                }
                assert(SharedClockProtocol::packFourLanes(rgb.data(), static_cast<uint16_t>(pixels), 255,
                    packed.data(), packed.size(), protocol));
                for (unsigned arm = 0; arm < 4; ++arm)
                    for (unsigned wirePixel = 0; wirePixel < pixels; ++wirePixel) {
                        const size_t byte = 4 + wirePixel * 4 + 1; // Blue byte on the wire.
                        unsigned value = 0;
                        for (unsigned bit = 0; bit < 8; ++bit)
                            value = (value << 1) | ((packed[byte * 8 + bit] >> arm) & 1);
                        if (wirePixel == radius) {
                            assert(value > 0); // First lit at hub, last lit at tip.
                            assert(value >= previousIntensity);
                            if (arm == 3) previousIntensity = value;
                        } else assert(value == 0);
                    }
            }
        }
}
static std::vector<uint8_t> fixture(unsigned channels = 6, unsigned frames = 2) {
    std::vector<uint8_t> bytes(32 + channels * frames);
    memcpy(bytes.data(), "PSEQ", 4);
    bytes[4] = 32; bytes[7] = 2; bytes[8] = 32; bytes[10] = static_cast<uint8_t>(channels);
    bytes[14] = static_cast<uint8_t>(frames); bytes[18] = 25;
    return bytes;
}
static void put(std::vector<uint8_t>& data, size_t at, uint32_t value, unsigned size) {
    for (unsigned i = 0; i < size; ++i) data[at + i] = static_cast<uint8_t>(value >> (i * 8));
}
static bool parse(const std::vector<uint8_t>& data, FseqHeader& header,
                  std::vector<SparseRange>& ranges, std::vector<CompressionBlock>& blocks) {
    FILE* file = fopen("build/native-fseq-fixture.bin", "w+b");
    assert(file);
    assert(fwrite(data.data(), 1, data.size(), file) == data.size());
    fflush(file);
    std::string error;
    bool result = readFseqMetadata(file, header, ranges, blocks, error);
    fclose(file);
    if (!result) assert(!error.empty());
    return result;
}
int main() {
    radialSequenceDirection();
    // Electrical bounce cannot change phase, speed, or the accepted index.
    HallIndexFilter hall;
    assert(hall.accept(1000000) && hall.averagePeriod() == 0);
    for (int64_t interval : {600000LL, 610000LL, 590000LL})
        assert(hall.accept(hall.last + interval));
    assert(hall.samples == 3 && hall.averagePeriod() == 600000);
    const auto stable = hall;
    for (int64_t noise : {1000LL, 3000LL, 49781LL, MinimumHallPeriodUs - 1}) {
        assert(!hall.accept(stable.last + noise));
        assert(hall.last == stable.last && hall.count == stable.count);
        assert(hall.averagePeriod() == stable.averagePeriod());
        assert(hall.lastPeriod == stable.lastPeriod && hall.samples == stable.samples);
    }
    assert(hall.rejected == 4 && hall.lastRejectedInterval == MinimumHallPeriodUs - 1);
    assert(hall.accept(stable.last + 630000));
    assert(hall.averagePeriod() == 610000); // Oldest sample rolls out.
    // The observed false 1,205 RPM edge must not make auto-duty go dark.
    assert(calculateDuty(hall.averagePeriod(), 256, 1471, 536).percent > 0);
    assert(!hall.accept(hall.last + 49781));
    assert(calculateDuty(hall.averagePeriod(), 256, 1471, 536).percent > 0);
    for (int64_t interval : {400000LL, 399000LL, 401000LL})
        assert(hall.accept(hall.last + interval));
    assert(hall.averagePeriod() == 400000); // 150 RPM with jitter.
    for (unsigned n = 0; n < 3; ++n) assert(hall.accept(hall.last + MinimumHallPeriodUs));
    assert(hall.averagePeriod() == MinimumHallPeriodUs);
    assert(hall.accept(hall.last + 10000000)); // Stop and restart.
    assert(hall.averagePeriod() == 0 && hall.samples == 0);
    assert(hall.accept(hall.last + 600000) && hall.averagePeriod() == 600000);
    // A first timestamp of zero is still an established index.
    HallIndexFilter zeroStart;
    assert(zeroStart.accept(0));
    assert(!zeroStart.accept(1000));
    assert(zeroStart.accept(600000) && zeroStart.averagePeriod() == 600000);
    HallIndexFilter slowStart;
    assert(slowStart.accept(100));
    assert(slowStart.accept(2000100) && slowStart.averagePeriod() == 2000000);
    // Compare the optimized packer with the independent byte-oriented wire
    // encoder, including lane isolation, brightness, tails and unaligned output.
    uint32_t randomState = 123456789;
    for (auto protocol : {SharedClockProtocol::Protocol::Apa102, SharedClockProtocol::Protocol::Sk9822})
        for (uint16_t pixels : {uint16_t(1), uint16_t(15), uint16_t(16), uint16_t(17),
                                uint16_t(72), uint16_t(144), uint16_t(1024)}) {
            const size_t clocks = SharedClockProtocol::frameBytes(pixels, protocol) * 8;
            std::vector<uint8_t> rgb(size_t(pixels) * 12), packed(clocks + 2, 0xA5);
            for (auto& value : rgb) {
                randomState = randomState * 1664525 + 1013904223;
                value = static_cast<uint8_t>(randomState >> 24);
            }
            for (uint8_t brightness : {uint8_t(0), uint8_t(1), uint8_t(25), uint8_t(127), uint8_t(255)}) {
                assert(!SharedClockProtocol::packFourLanes(rgb.data(), pixels, brightness, packed.data() + 1, clocks - 1, protocol));
                assert(SharedClockProtocol::packFourLanes(rgb.data(), pixels, brightness, packed.data() + 1, clocks, protocol));
                assert(packed.front() == 0xA5 && packed.back() == 0xA5);
                for (unsigned lane = 0; lane < 4; ++lane)
                    for (size_t byte = 0; byte < clocks / 8; ++byte) {
                        uint8_t received = 0;
                        for (unsigned bit = 0; bit < 8; ++bit) {
                            const uint8_t levels = packed[1 + byte * 8 + bit];
                            assert(!(levels & 0xF0));
                            received = uint8_t((received << 1) | ((levels >> lane) & 1));
                        }
                        assert(received == SharedClockProtocol::wireByte(rgb.data() + size_t(lane) * pixels * 3,
                            pixels, byte, brightness, false, protocol));
                    }
            }
        }
    RadialGainCache<144> gains;
    for (unsigned pixels : {1U, 72U, 144U, 256U})
        for (unsigned regular : {0U, 10U, 50U, 100U})
            for (unsigned center : {0U, 3U, 10U, 100U})
                for (bool enabled : {false, true})
                    for (bool inputAtHub : {false, true})
                        for (unsigned pixel = 0; pixel < pixels; ++pixel) {
                            const auto expected = radialColor({255, 101, 17}, pixel, pixels,
                                regular, center, enabled, inputAtHub);
                            const uint16_t gain = gains.gain(pixel, pixels, regular, center, enabled, inputAtHub);
                            const std::array<uint8_t, 3> actual{uint8_t((255 * gain + 128) >> 8),
                                uint8_t((101 * gain + 128) >> 8), uint8_t((17 * gain + 128) >> 8)};
                            assert(actual == expected);
                        }
    // A flash is two complete protocol frames in one DMA buffer. Decode each
    // lane independently, including the second frame's brightness headers and
    // tail clocks: its RGB payload must always be black.
    for (auto protocol : {SharedClockProtocol::Protocol::Apa102, SharedClockProtocol::Protocol::Sk9822})
        for (uint16_t pixels : {uint16_t(72), uint16_t(144)}) {
            const size_t clocks = SharedClockProtocol::frameBytes(pixels, protocol) * 8;
            std::vector<uint8_t> rgb(size_t(pixels) * 4 * 3, 255), burst(clocks * 2 + 2, 0xA5);
            assert(SharedClockProtocol::packFourLanes(rgb.data(), pixels, 255, burst.data() + 1, clocks, protocol));
            assert(!SharedClockProtocol::packBlackFourLanes(pixels, burst.data() + 1 + clocks, clocks - 1, protocol));
            assert(SharedClockProtocol::packBlackFourLanes(pixels, burst.data() + 1 + clocks, clocks, protocol));
            assert(burst.front() == 0xA5 && burst.back() == 0xA5);
            for (unsigned lane = 0; lane < 4; ++lane)
                for (unsigned frame = 0; frame < 2; ++frame)
                    for (size_t byte = 0; byte < clocks / 8; ++byte) {
                        uint8_t received = 0;
                        for (unsigned bit = 0; bit < 8; ++bit)
                            received = uint8_t((received << 1) | ((burst[1 + frame * clocks + byte * 8 + bit] >> lane) & 1));
                        if (byte < 4) assert(received == 0);
                        else if (byte < 4 + size_t(pixels) * 4)
                            assert(received == ((byte - 4) % 4 == 0 || frame == 0 ? 255 : 0));
                        else if (protocol == SharedClockProtocol::Protocol::Sk9822 && byte < 8 + size_t(pixels) * 4)
                            assert(received == 0);
                        else assert(received == 255);
                    }
            for (unsigned colorFrames : {1U, 2U, 3U}) {
                const int64_t duration = (clocks * (colorFrames + 1) * 1000000 + PlaybackDmaClockHz - 1) / PlaybackDmaClockHz;
                for (int64_t slot : {1562LL, 2343LL}) {
                    assert(preparedPulseFits(slot - duration - 100, slot, static_cast<uint32_t>(clocks), colorFrames));
                    assert(!preparedPulseFits(slot - duration - 99, slot, static_cast<uint32_t>(clocks), colorFrames));
                    assert(!preparedPulseFits(slot, slot, static_cast<uint32_t>(clocks), colorFrames));
                    assert(!preparedPulseFits(0, 0, static_cast<uint32_t>(clocks), colorFrames));
                }
            }
            assert(!preparedPulseFits(0, 10000, static_cast<uint32_t>(clocks), 0));
            assert(!preparedPulseFits(0, 10000, static_cast<uint32_t>(clocks), 4));
        }
    const std::array<uint32_t, 4> sharedStarts{1, 1, 1, 1};
    FseqHeader fileDefaults;
    fileDefaults.major = 2; fileDefaults.frames = 600; fileDefaults.stepMs = 50;
    for (unsigned count : {1U, 4U, 80U, 160U, 190U, 256U}) {
        fileDefaults.channels = count * 144 * 3;
        for (unsigned arms : {1U, 2U, 3U, 4U}) {
            const auto derived = deriveFseqSettings(fileDefaults, {}, 144, arms, sharedStarts);
            assert(derived.frameUs == 50000 && derived.spokes == count && derived.channelsPerSpoke == 432);
            // Physical arms sample one shared image; never divide its width
            // by arm count. Four virtual spokes still need angular indexing.
            for (unsigned spoke = 0; spoke < count; ++spoke)
                for (unsigned arm = 0; arm < arms; ++arm) {
                    const auto channel = fseqImageChannel(sharedStarts[arm], spoke, 143, derived.channelsPerSpoke);
                    assert(channel == uint64_t(spoke) * 432 + 429);
                    assert(channel + 2 < fileDefaults.channels);
                }
        }
    }
    for (unsigned step : {9U, 10U, 25U, 33U, 50U, 255U}) {
        fileDefaults.stepMs = static_cast<uint8_t>(step);
        const auto derived = deriveFseqSettings(fileDefaults, {}, 144, 4, sharedStarts);
        assert(derived.frameUs == step * 1000);
        // After 1,000 frames a 33 ms export remains exactly 33 seconds.
        assert(uint64_t(derived.frameUs) * 1000 == uint64_t(step) * 1000000);
    }
    for (unsigned step : {0U, 1U, 5U, 8U}) {
        fileDefaults.stepMs = static_cast<uint8_t>(step);
        const auto derived = deriveFseqSettings(fileDefaults, {}, 144, 4, sharedStarts);
        assert(!derived.frameUs && derived.spokes == 256);
    }
    fileDefaults.stepMs = 25;
    assert(!deriveFseqSettings(fileDefaults, {}, 144, 4, {1, 433, 865, 1297}).spokes);
    assert(deriveFseqSettings(fileDefaults, {}, 144, 4, {1, 433, 865, 1297}).frameUs == 25000);
    assert(!deriveFseqSettings(fileDefaults, {}, 144, 4, {2, 2, 2, 2}).spokes);
    assert(!deriveFseqSettings(fileDefaults, {}, 0, 4, sharedStarts).spokes);
    assert(!deriveFseqSettings(fileDefaults, {}, 144, 0, sharedStarts).spokes);
    assert(!deriveFseqSettings({}, {}, 144, 4, sharedStarts).frameUs);
    ++fileDefaults.channels;
    assert(!deriveFseqSettings(fileDefaults, {}, 144, 4, sharedStarts).spokes);
    --fileDefaults.channels;
    const std::array<uint32_t, 4> sparseStarts{1001, 1001, 1001, 1001};
    const auto sparseDefaults = deriveFseqSettings(fileDefaults, {{1000, 110592, 0}}, 144, 4, sparseStarts);
    assert(sparseDefaults.spokes == 256);
    assert(fseqImageChannel(1001, 255, 143, sparseDefaults.channelsPerSpoke) + 2 == 111591);
    // Adjacent sparse ranges may arrive out of order; holes cannot be inferred.
    assert(deriveFseqSettings(fileDefaults, {{1050, 110542, 50}, {1000, 50, 0}}, 144, 4, sparseStarts).spokes == 256);
    assert(!deriveFseqSettings(fileDefaults, {{1051, 110542, 50}, {1000, 50, 0}}, 144, 4, sparseStarts).spokes);
    assert(!deriveFseqSettings(fileDefaults, {{1000, 110592, 0}}, 144, 4, sharedStarts).spokes);
    fileDefaults.channels = 65536 * 3;
    assert(!deriveFseqSettings(fileDefaults, {}, 1, 4, sharedStarts).spokes);
    // Auto duty covers the paint window (including wake margin) AND the dark
    // transfer, rounds upward, and uses continuous output only when necessary.
    for (unsigned spokes : {1U, 40U, 80U, 160U, 65535U})
        for (int64_t period : {150000LL, 500000LL, 1000000LL, 3000000LL})
            for (uint32_t budget : {300U, 1000U, 1500U, 3000U, 10000U}) {
                const auto duty = calculateDuty(period, spokes, budget);
                assert(duty.available);
                if (!duty.feasible) {
                    assert(duty.percent == 0 && duty.spokeUs < budget + PaintWakeMarginUs);
                    continue;
                }
                assert(duty.percent >= 1 && duty.percent <= 100);
                assert(duty.spokeUs * duty.percent / 100 >= budget + PaintWakeMarginUs);
                assert(spokeGate(0, duty.spokeUs, duty.percent, false, 0, spokes, budget).fits);
                if (duty.continuous) {
                    assert(duty.percent == 100);
                    for (unsigned lower = 1; lower < 100; ++lower)
                        assert(duty.spokeUs * lower / 100 < budget + PaintWakeMarginUs ||
                            duty.spokeUs * (100 - lower) / 100 < budget);
                } else {
                    assert(duty.spokeUs * (100 - duty.percent) / 100 >= budget);
                    assert(duty.spokeUs * (duty.percent - 1) / 100 < budget + PaintWakeMarginUs);
                }
            }
    assert(!calculateDuty(0, 80, 1000).available);
    assert(!calculateDuty(-1, 80, 1000).available);
    assert(!calculateDuty(500000, 0, 1000).available);
    assert(!calculateDuty(500000, 80, 0).available);
    assert(calculateDuty(500000, 160, 1471, 536).percent == 56); // 120 RPM, cached black transfer.
    assert(calculateDuty(1000000, 160, 1471, 536).percent == 28); // Slowing halves the duty.
    assert(calculateDuty(500000, 160, 1471).continuous); // Full color transfer in the gap does not fit.
    assert(calculateDuty(250000, 160, 1471).percent == 0); // Not enough room even at 100%.
    assert(calculateDuty(500000, 160, 2000).continuous);
    // Synchronized arms can use the cached black frame; a mixed frame cannot.
    const auto asymmetric = calculateDuty(500000, 160, 1701, 536);
    assert(asymmetric.percent == 63 && !asymmetric.continuous);
    assert(spokeGate(0, asymmetric.spokeUs, asymmetric.percent, false, 0, 160,
        asymmetric.transferUs, asymmetric.blankUs).fits);
    assert(!spokeGate(0, asymmetric.spokeUs, asymmetric.percent, false, 0, 160,
        asymmetric.transferUs).fits);
    // Regression: a 4-arm preparation can fit the auto-calculated window even
    // though restarting the entire paint budget before EACH arm rejects all
    // four. Use the observed 80-spoke, ~169 RPM configuration and real delays.
    const auto minimumPaint = calculateDuty(355968, 80, 874, 554);
    assert(minimumPaint.percent == 26 && !minimumPaint.continuous);
    const int64_t paintDeadline = static_cast<int64_t>(minimumPaint.spokeUs * minimumPaint.percent / 100);
    constexpr int64_t woke = 220, beforePixels = 70, perArm = 45;
    assert(spokeGate(woke / minimumPaint.spokeUs, minimumPaint.spokeUs, minimumPaint.percent,
        false, 0, 80, 874, 554).canStart);
    for (unsigned arm = 0; arm < 4; ++arm) {
        const int64_t at = woke + beforePixels + arm * perArm;
        assert(at + 874 > paintDeadline); // Old guard leaves this arm black.
        assert(remainingPaintFits(woke, at, paintDeadline, 874, 674));
    }
    assert(woke + beforePixels + 4 * perArm + 674 <= paintDeadline);
    // Real overruns still get rejected, even after most preparation is done.
    assert(!remainingPaintFits(woke, paintDeadline - 673, paintDeadline, 874, 674));
    assert(!remainingPaintFits(woke, woke, woke + 873, 874, 674));
    assert(!remainingPaintFits(woke, woke, 0, 874, 674));
    for (double angle : {-270.0, -180.0, -90.0, 0.0, 90.0, 180.0, 270.0})
        for (unsigned spokes : {4U, 40U, 80U, 160U}) assert(sameSpokeBoundary(angle, spokes));
    assert(!sameSpokeBoundary(90, 77));
    assert(!sameSpokeBoundary(90.1, 160));
    assert(sameSpokeBoundary(92.25, 160)); // A whole-spoke trim stays synchronized.
    RecentTransferPeak peak;
    assert(peak.peak(0) == 0);
    peak.observe(0, 1000);peak.observe(100, 2000);peak.observe(499999, 900);
    peak.observe(500000, 1200);
    assert(peak.peak(1999999) == 2000);
    assert(peak.peak(2000000) == 1200);
    assert(peak.peak(2500000) == 0); // An outlier expires even while lights are blank.
    peak.observe(10000000, 1100);
    assert(peak.peak(10000000) == 1100);
    // Alignment is an exact green half with three separate red rays. Boundaries
    // are half-open so every angle belongs to one segment, including transitions.
    const std::array<uint8_t, 3> green{0, 255, 0}, red{255, 0, 0}, black{};
    unsigned greenSamples = 0, redSamples = 0, redRuns = 0;
    auto previousAlignment = black;
    for (unsigned sample = 0; sample < 3600; ++sample) {
        const double angle = sample / 10.0;
        const auto segment = alignmentSegment(angle);
        assert(angle >= segment.start && angle < segment.end);
        if (segment.color == green) ++greenSamples;
        else if (segment.color == red) {
            ++redSamples;
            if (previousAlignment != red) ++redRuns;
        } else assert(segment.color == black);
        previousAlignment = segment.color;
    }
    assert(greenSamples == 1800 && redSamples == 270 && redRuns == 3);
    assert(alignmentSegment(179.999).color == green && alignmentSegment(180).color == black);
    for (double center : {225.0, 270.0, 315.0}) {
        assert(alignmentSegment(center - 4.5).color == red);
        assert(alignmentSegment(center).color == red);
        assert(alignmentSegment(center + 4.5).color == black);
    }
    // At the same physical location every arm paints the same reference,
    // including non-integer offsets, negative trims, both wiring directions,
    // and phase wrapping. Playback uses this same position calculation.
    for (bool armCw : {false, true}) for (bool rotorCw : {false, true})
        for (double offset : {0.0, 0.1, 37.5, -123.4, 359.9})
            for (unsigned arm = 0; arm < 4; ++arm)
                for (unsigned sample = 0; sample < 1440; ++sample) {
                    const double angle = sample / 4.0 + 0.125;
                    const double phase = offset + armAngleDegrees(arm, armCw, rotorCw) - 0.4;
                    const int64_t elapsed = std::llround((angle - phase + 1080) * 100000);
                    const double position = spokePosition(elapsed, 36000000, 360, phase);
                    assert(std::abs(position - angle) < 0.000001);
                    assert(alignmentSegment(position).color == alignmentSegment(angle).color);
                }
    // Red windows fit ordinary 400 RPM / 144-pixel transfers. At extreme
    // speeds an impossible red window is rejected rather than smeared.
    assert(spokeGate(0, 150000 * 9.0 / 360, 100, false, 0, 1, 1500).canStart);
    assert(!spokeGate(0, 50000 * 9.0 / 360, 100, false, 0, 1, 1500).fits);
    // Match IDF's integer request -> prescaler -> reported Hz calculation.
    for (auto hz : StripClockRates) {
        assert(validStripClock(hz));
        const auto divider = 80000000U / hz;
        assert(divider >= 2 && divider <= 64);
        assert(80000000U / divider == hz);
    }
    assert(validStripClock(26666666U) && 80000000U / 26666666U == 3);
    for (auto hz : {0U, 1000000U, 3000000U, 24000000U, 26666667U, 26670000U, 30000000U, 32000000U, 80000000U})
        assert(!validStripClock(hz));
    const std::array<uint8_t, 3> expectedSolids[] = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}
    };
    for (unsigned phase = 0; phase < 4; ++phase)
        for (unsigned arm = 0; arm < 4; ++arm)
            for (unsigned pixel = 0; pixel < 144; ++pixel) {
                assert(speedPatternColor(phase * 2000, pixel, arm, 144) == expectedSolids[phase]);
                assert(speedPatternColor(phase * 2000 + 1999, pixel, arm, 144) == expectedSolids[phase]);
            }
    assert(speedPatternPhase(12000) == 0);
    assert(speedPatternColor(12000, 0, 0, 144) == expectedSolids[0]);
    // Distinct arm/block colors and moving marker; then the alternating-bit
    // stress pattern changes every pixel after 100 ms and repeats after 200 ms.
    for (unsigned arm = 0; arm < 4; ++arm) {
        for (unsigned block = 0; block < 4; ++block)
            assert(speedPatternColor(8000, block * 8, arm, 144) == expectedSolids[(block + arm) % 4]);
        assert(speedPatternColor(8000, 80, arm, 144) == expectedSolids[3]);
        assert(speedPatternColor(8100, 81, arm, 144) == expectedSolids[3]);
        for (unsigned pixel = 0; pixel < 144; ++pixel) {
            assert(speedPatternColor(10000, pixel, arm, 144) != speedPatternColor(10100, pixel, arm, 144));
            assert(speedPatternColor(10000, pixel, arm, 144) == speedPatternColor(10200, pixel, arm, 144));
        }
    }
    const std::array<uint8_t, 3> sampleColor{255, 100, 50};
    for (unsigned pixels : {1U, 2U, 5U, 144U, 1024U}) {
        unsigned previous = 256;
        for (unsigned pixel = 0; pixel < pixels; ++pixel) {
            assert(radialColor(sampleColor, pixel, pixels, 50, 10, false) == sampleColor);
            assert(radialColor(sampleColor, pixel, pixels, 50, 100, true) == sampleColor);
            assert((radialColor(sampleColor, pixel, pixels, 0, 10, true) == std::array<uint8_t, 3>{}));
            const unsigned gain = radialBrightnessGain(pixel, pixels, 50, 10, true);
            assert(gain <= previous); previous = gain;
        }
        assert(radialColor(sampleColor, 0, pixels, 50, 10, true) == sampleColor);
        if (pixels > 1) {
            assert((radialColor(sampleColor, pixels - 1, pixels, 50, 10, true) == std::array<uint8_t, 3>{51, 20, 10}));
            assert((radialColor(sampleColor, pixels - 1, pixels, 50, 0, true) == std::array<uint8_t, 3>{}));
        }
    }
    // Absolute endpoints: center 10%, regular/tip 50%, with a linear ramp.
    // Wire reversal changes which end is sent first, not the radial brightness.
    std::vector<uint8_t> gradient;
    for (unsigned pixel = 0; pixel < 5; ++pixel) {
        const auto rgb = radialColor({255, 255, 255}, pixel, 5, 50, 10, true);
        gradient.insert(gradient.end(), rgb.begin(), rgb.end());
    }
    const unsigned expectedWire[] = {127, 102, 76, 51, 25};
    for (auto protocol : {SharedClockProtocol::Protocol::Apa102, SharedClockProtocol::Protocol::Sk9822}) {
        for (unsigned pixel = 0; pixel < 5; ++pixel) {
            const auto at = 5 + pixel * 4;
            assert(SharedClockProtocol::wireByte(gradient.data(), 5, at, 127, false, protocol) == expectedWire[pixel]);
            assert(SharedClockProtocol::wireByte(gradient.data(), 5, at, 127, true, protocol) == expectedWire[4 - pixel]);
        }
    }
    // Actual strips receive data at the hub. Independent image reversal must
    // never move the dim end to the tip. Check the transmitted LED values.
    for (bool reverseImage : {false, true}) {
        for (bool inputAtHub : {false, true}) {
            std::array<uint8_t, 15> wireRgb{};
            for (unsigned pixel = 0; pixel < 5; ++pixel) {
                const unsigned wirePixel = reverseImage ? 4 - pixel : pixel;
                const auto rgb = radialColor({255, 255, 255}, wirePixel, 5, 50, 10, true, inputAtHub);
                std::copy(rgb.begin(), rgb.end(), wireRgb.begin() + wirePixel * 3);
            }
            for (unsigned pixel = 0; pixel < 5; ++pixel)
                assert(SharedClockProtocol::wireByte(wireRgb.data(), 5, 5 + pixel * 4, 127) ==
                    expectedWire[inputAtHub ? 4 - pixel : pixel]);
        }
    }
    const std::array<uint8_t, 3> spokeColors[] = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}
    };
    // Observe both arm orders and both viewing-side rotation directions.
    // each fixed disk position as four different arms pass it at different
    // times. Every pass must paint the same spoke/color. This catches the old
    // clockwise-offset assumption that swapped red/blue and green/white on
    // alternate arms, even though all their paint windows were aligned.
    for (bool armClockwise : {false, true}) {
    for (bool rotationClockwise : {false, true}) {
    for (unsigned revolution : {0U, 1U, 10000U}) {
        for (unsigned spoke = 0; spoke < 80; ++spoke) {
            for (unsigned arm = 0; arm < 4; ++arm) {
                const unsigned arrivalOffset = armClockwise == rotationClockwise
                    ? (80 - arm * 20) % 80 : arm * 20;
                const unsigned arrivalSpoke = (spoke + arrivalOffset) % 80;
                const int64_t elapsed = int64_t(revolution) * 500000 + arrivalSpoke * 6250 + 3125;
                const double position = spokePosition(elapsed, 500000, 80,
                    armAngleDegrees(arm, armClockwise, rotationClockwise));
                assert(std::abs(position - (spoke + 0.5)) < 0.000001);
                assert(spokeColor(SpokePattern::Quarters, unsigned(position), 80) == spokeColors[spoke / 20]);
                assert(spokeColor(SpokePattern::Alternating, unsigned(position), 80) == spokeColors[spoke % 4]);
                // Calibrations add to physical geometry instead of replacing it.
                const double calibrated = spokePosition(elapsed, 500000, 80,
                    armAngleDegrees(arm, armClockwise, rotationClockwise) + 9 - 4.5);
                assert(std::abs(calibrated - ((spoke + 1) % 80 + 0.5)) < 0.000001);
            }
        }
    }
    }
    }
    for (unsigned arm = 0; arm < 4; ++arm) {
        assert(armAngleDegrees(arm, false, true) == -90 * int(arm)); // Existing geometry.
        assert(armAngleDegrees(arm, false, true) == armAngleDegrees(arm, true, false)); // Other viewing side.
        assert(armOrderTestArm(arm * 1200000, 4) == int(arm));
        assert(armOrderTestArm(arm * 1200000 + 999999, 4) == int(arm));
        assert(armOrderTestArm(arm * 1200000 + 1000000, 4) == -1);
        assert(armOrderTestArm(arm * 1200000 + 1199999, 4) == -1);
    }
    assert(armOrderTestArm(4800000, 4) == 0 && armOrderTestArm(3600000, 3) == 0);
    assert(armOrderTestArm(-1, 4) == -1 && armOrderTestArm(0, 2) == -1);
    assert(armOrderTestArm(ArmOrderTestDurationUs, 4) == -1);
    assert(spokePosition(0, 500000, 80, -4.5) == 79);
    assert(spokePosition(0, 500000, 80, 364.5) == 1);
    assert(spokePosition(500000, 500000, 80, 90) == 20);
    assert(spokePosition(0, 0, 80, 0) == 0);
    assert((spokeColor(SpokePattern::Quarters, 0, 0) == std::array<uint8_t, 3>{}));
    // Remainders stay covered, with at most one extra spoke per quarter.
    for (unsigned count : {4U, 77U, 80U, 81U, 65535U}) {
        unsigned totals[4] = {};
        for (unsigned spoke = 0; spoke < count; ++spoke) {
            const auto color = spokeColor(SpokePattern::Quarters, spoke, count);
            for (unsigned i = 0; i < 4; ++i) if (color == spokeColors[i]) ++totals[i];
            assert(spokeColor(SpokePattern::Alternating, spoke, count) == spokeColors[spoke % 4]);
        }
        assert(totals[0] + totals[1] + totals[2] + totals[3] == count);
        for (unsigned total : totals) assert(total == count / 4 || total == (count + 3) / 4);
        for (auto pattern : {SpokePattern::Quarters, SpokePattern::Alternating}) {
            assert(spokeColor(pattern, count, count) == spokeColors[0]);
            assert(spokeColor(pattern, count * 123 + 1, count) == spokeColor(pattern, 1, count));
        }
    }
    const std::array<uint8_t, 3> fadeColors[] = {
        {255, 0, 0}, {255, 255, 0}, {0, 255, 0},
        {0, 255, 255}, {0, 0, 255}, {255, 0, 255}
    };
    for (unsigned i = 0; i < 6; ++i) assert(colorFadeAt(i * 2000) == fadeColors[i]);
    // Check continuity across segment boundaries and the cycle wrap, plus
    // periodicity after long uptime, rather than only the six endpoint colors.
    for (uint64_t ms = 0; ms < ColorFadeCycleMs; ms += ColorFadeFrameMs) {
        const auto color = colorFadeAt(ms), next = colorFadeAt(ms + ColorFadeFrameMs);
        assert(color == colorFadeAt(ms + uint64_t(ColorFadeCycleMs) * 1000000));
        for (unsigned c = 0; c < 3; ++c) assert(std::abs(int(next[c]) - int(color[c])) <= 3);
    }
    // Existing installations and SD backups adopt the new default without
    // replacing a custom name or another device's generated name.
    const uint8_t mac[] = {0x02, 0xab, 0xcd, 0xef, 0x12, 0x34};
    assert(hostnameAfterUpgrade("", mac) == "lpov");
    assert(hostnameAfterUpgrade("pov-ef1234", mac) == "lpov");
    assert(hostnameAfterUpgrade("pov-CDAB02", mac) == "lpov");
    assert(hostnameAfterUpgrade("lpov", mac) == "lpov");
    assert(hostnameAfterUpgrade("Workshop-Spinner", mac) == "Workshop-Spinner");
    assert(hostnameAfterUpgrade("pov-feed00", mac) == "pov-feed00");
    FseqHeader h;
    std::vector<SparseRange> ranges;
    std::vector<CompressionBlock> blocks;
    auto plain = fixture();
    assert(parse(plain, h, ranges, blocks));
    assert(h.channels == 6 && h.frames == 2 && h.stepMs == 25 && !h.compression);
    assert(sparseOffset(5, h.channels, ranges) == 5);
    assert(sparseOffset(6, h.channels, ranges) == -1);
    auto truncated = plain; truncated.pop_back(); assert(!parse(truncated, h, ranges, blocks));
    auto bad = plain; bad[7] = 3; assert(!parse(bad, h, ranges, blocks));
    bad = plain; bad[10] = 0; assert(!parse(bad, h, ranges, blocks));
    bad = plain; put(bad, 14, UINT32_MAX, 4); assert(!parse(bad, h, ranges, blocks));

    auto sparse = fixture(); sparse.insert(sparse.begin() + 32, 12, 0);
    sparse[4] = sparse[8] = 44; sparse[22] = 2;
    put(sparse, 32, 100, 3); put(sparse, 35, 2, 3);
    put(sparse, 38, 200, 3); put(sparse, 41, 4, 3);
    assert(parse(sparse, h, ranges, blocks));
    assert(sparseOffset(100, h.channels, ranges) == 0);
    assert(sparseOffset(101, h.channels, ranges) == 1);
    assert(sparseOffset(102, h.channels, ranges) == -1);
    assert(sparseOffset(200, h.channels, ranges) == 2);
    assert(sparseOffset(203, h.channels, ranges) == 5);
    bad = sparse; put(bad, 38, 101, 3); assert(!parse(bad, h, ranges, blocks));
    bad = sparse; put(bad, 41, 3, 3); assert(!parse(bad, h, ranges, blocks));
    bad = sparse; bad[4] = 32; assert(!parse(bad, h, ranges, blocks));

    // Two compressed blocks, covering frames [0,2) and [2,4).
    // Metadata validation is independent of zlib payload decompression.
    auto compressed = fixture(6, 4); compressed.resize(56);
    compressed[4] = compressed[8] = 48; compressed[20] = 2; compressed[21] = 2;
    put(compressed, 32, 0, 4); put(compressed, 36, 4, 4);
    put(compressed, 40, 2, 4); put(compressed, 44, 4, 4);
    assert(parse(compressed, h, ranges, blocks));
    assert(blocks.size() == 2 && blocks[0].firstFrame == 0 && blocks[1].firstFrame == 2);
    assert(blocks[0].fileOffset == 48 && blocks[1].fileOffset == 52);
    bad = compressed; put(bad, 32, 6, 4); assert(!parse(bad, h, ranges, blocks));
    bad = compressed; put(bad, 40, 0, 4); assert(!parse(bad, h, ranges, blocks));
    bad = compressed; bad.pop_back(); assert(!parse(bad, h, ranges, blocks));
    bad = compressed; bad[20] = 1; assert(!parse(bad, h, ranges, blocks));

    // FSEQ 2.1 extends the block count to 12 bits; allow zero-length padding.
    auto extended = fixture(6, 4); extended.resize(32 + 256 * 8 + 4);
    put(extended, 4, 32 + 256 * 8, 2); put(extended, 8, 32 + 256 * 8, 2);
    extended[6] = 1; extended[20] = 0x12; extended[21] = 0;
    put(extended, 32, 0, 4); put(extended, 36, 4, 4);
    assert(parse(extended, h, ranges, blocks));
    assert(blocks.size() == 1 && blocks[0].fileOffset == 2080);

    static_assert(BoardPins::outputPinsValid());
    // Designer-confirmed wiring: LED traffic must never reach motor speed PWM.
    static_assert(BoardPins::Clock == 42 && BoardPins::MotorSpeedPwm == 1);
    for (int pin : BoardPins::OutputData) assert(pin != BoardPins::MotorSpeedPwm);
    static_assert(BoardPins::InitialArms == 4 && BoardPins::PulsesPerRevolution == 1);
    static_assert(SharedClockProtocol::frameBytes(144) * 8 == 4744);
    const uint8_t rgb[] = {17, 34, 51};
    assert(SharedClockProtocol::wireByte(rgb, 1, 5, 255) == 51);
    assert(SharedClockProtocol::wireByte(rgb, 1, 7, 255) == 17);
    assert(SharedClockProtocol::wireByte(rgb, 1, 5, 0) == 0);
    assert(SharedClockProtocol::wireByte(rgb, 1, 8, 255) == 0);
    assert(SharedClockProtocol::wireByte(rgb, 1, 12, 255) == 255);
    // Decode the parallel DMA stream as four actual rising-edge receivers.
    // Cover end-frame rounding, DMA descriptor boundaries and maximum geometry.
    const uint16_t geometries[] = {1, 15, 16, 17, 144, 1024};
    const uint8_t levels[] = {0, 31, 255};
    for (auto protocol : {SharedClockProtocol::Protocol::Sk9822, SharedClockProtocol::Protocol::Apa102}) {
    for (uint16_t pixels : geometries) {
        std::vector<uint8_t> strips(size_t(pixels) * 4 * 3);
        for (size_t i = 0; i < strips.size(); ++i) strips[i] = uint8_t(i * 73 + i / 3 * 11);
        const size_t clocks = SharedClockProtocol::frameBytes(pixels, protocol) * 8;
        for (uint8_t brightness : levels) {
            std::vector<uint8_t> packed(clocks + 2, 0xA5);
            assert(!SharedClockProtocol::packFourLanes(strips.data(), pixels, brightness, packed.data() + 1, clocks - 1, protocol));
            for (uint8_t v : packed) assert(v == 0xA5);
            assert(SharedClockProtocol::packFourLanes(strips.data(), pixels, brightness, packed.data() + 1, clocks, protocol));
            assert(packed.front() == 0xA5 && packed.back() == 0xA5);
            for (size_t bit = 1; bit <= clocks; ++bit) assert((packed[bit] & 0xF0) == 0);
            for (unsigned lane = 0; lane < 4; ++lane) {
                std::vector<uint8_t> received(clocks / 8);
                for (size_t bit = 0; bit < clocks; ++bit)
                    received[bit / 8] = uint8_t((received[bit / 8] << 1) | ((packed[bit + 1] >> lane) & 1));
                for (unsigned byte = 0; byte < 4; ++byte) assert(received[byte] == 0);
                for (unsigned pixel = 0; pixel < pixels; ++pixel) {
                    assert(received[4 + pixel * 4] == 255);
                    for (unsigned component = 0; component < 3; ++component) {
                        const unsigned source = strips[(lane * pixels + pixel) * 3 + component];
                        assert(received[4 + pixel * 4 + 3 - component] == source * (unsigned(brightness) + 1) / 256);
                    }
                }
                const size_t resetEnd = 4 + size_t(pixels) * 4 + (protocol == SharedClockProtocol::Protocol::Sk9822 ? 4 : 0);
                for (size_t byte = 4 + size_t(pixels) * 4; byte < resetEnd; ++byte) assert(received[byte] == 0);
                for (size_t byte = resetEnd; byte < received.size(); ++byte) assert(received[byte] == 255);
            }
        }
    }
    }
    static_assert(SharedClockProtocol::frameBytes(144, SharedClockProtocol::Protocol::Apa102) * 8 == 4712);
    // An absent router must not keep scanning while someone uses POV-Spinner.
    WifiRetry retry;
    retry.reset(true, false, 0);
    assert(!retry.ready(499999, 0));
    assert(!retry.ready(1000000, 1));
    assert(retry.ready(1000000, 0));
    retry.started(); retry.failed(1000000);
    assert(!retry.ready(5999999, 0) && retry.ready(6000000, 0));
    retry.started(); retry.failed(6000000);
    assert(!retry.ready(20999999, 0) && retry.ready(21000000, 0));
    retry.started(); retry.failed(21000000);
    assert(!retry.ready(1000000000, 0)); // No infinite retry loop.
    retry.reset(true, true, 0); // Explicit retry permits exactly one attempt with a local client.
    assert(retry.ready(500000, 1));
    retry.started(); retry.failed(1000000);
    assert(!retry.ready(6000000, 1) && retry.ready(6000000, 0));
    retry.connected(); retry.failed(10000000);
    assert(retry.attempts == 0 && retry.ready(15000000, 0));
    retry.reset(false, true, 0); // Forget network disables even manual attempts.
    assert(!retry.ready(1000000000, 0));
    std::cout << "Native format, LED/DMA protocol, radial brightness, spatial spoke patterns, color fade, hostname migration and Wi-Fi retry policy tests passed\n";
}
