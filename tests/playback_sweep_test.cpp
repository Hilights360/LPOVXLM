#include "fseq.hpp"
#include "playback_timing.hpp"
#include "arm_wiring.hpp"
#include <cassert>
#include <iostream>

using namespace pov;
namespace {
constexpr unsigned Spokes = 160, Pixels = 32, Frames = 100;
constexpr int64_t PeriodUs = 480000, FrameUs = 25000;
constexpr unsigned Channels = Spokes * Pixels * 3;

void ringFixture(const char* path) {
    std::vector<uint8_t> data(32 + Channels * Frames);
    memcpy(data.data(), "PSEQ", 4);
    data[4] = data[8] = 32; data[7] = 2;
    for (unsigned byte = 0; byte < 4; ++byte) {
        data[10 + byte] = static_cast<uint8_t>(Channels >> (byte * 8));
        data[14 + byte] = static_cast<uint8_t>(Frames >> (byte * 8));
    }
    data[18] = 25;
    for (unsigned frame = 0; frame < Frames; ++frame)
        for (unsigned spoke = 0; spoke < Spokes; ++spoke) {
            const unsigned radius = frame % (Pixels - 2);
            data[32 + frame * Channels + (spoke * Pixels + radius) * 3] = 255;
            data[32 + frame * Channels + (spoke * Pixels + radius + 1) * 3] = 255;
        }
    FILE* file = fopen(path, "wb"); assert(file);
    assert(fwrite(data.data(), 1, data.size(), file) == data.size());
    fclose(file);
}

unsigned simulate(const char* path, bool latchSweeps, bool clockwiseArms, bool delayedReads) {
    Fseq seq; std::string error;
    assert(seq.open("/expanding-ring.fseq", path, error));
    PlaybackSweep sweep;
    uint32_t decoded = 0;
    unsigned tornImages = 0;
    std::array<int, Spokes> radii;
    radii.fill(-1);
    // Four complete turns at 125 RPM, with real frame data arriving at 40 FPS.
    // Rendering occurs every 3 ms; file deadlines generally fall between spokes.
    for (int64_t now = 0; now < PeriodUs * 4; now += PeriodUs / Spokes) {
        const uint32_t due = static_cast<uint32_t>(now / FrameUs);
        const bool stalled = delayedReads && now >= 600000 && now < 850000;
        if (due != decoded && !stalled) {
            auto job = seq.prepare(due);
            assert(job.run(error) && seq.publish(job));
            decoded = due;
            if (!latchSweeps) assert(seq.present()); // Original mid-sweep behavior.
        }
        const bool boundary = sweep.boundary(static_cast<uint32_t>(now / PeriodUs),
            now % PeriodUs, PeriodUs, 4, 0);
        if (boundary && latchSweeps) seq.present();
        for (unsigned arm = 0; arm < 4; ++arm) {
            const unsigned spoke = static_cast<unsigned>(spokePosition(now % PeriodUs, PeriodUs,
                Spokes, armAngleDegrees(arm, clockwiseArms, true)));
            assert(spoke < Spokes && radii[spoke] == -1);
            unsigned radius = 0;
            while (radius < Pixels && !seq.channel((spoke * Pixels + radius) * 3)) ++radius;
            assert(radius + 1 < Pixels);
            assert(seq.channel((spoke * Pixels + radius + 1) * 3) == 255);
            radii[spoke] = static_cast<int>(radius);
        }
        if ((now / (PeriodUs / Spokes) + 1) % (Spokes / 4) == 0) {
            for (int radius : radii) assert(radius >= 0); // Every image column painted.
            const auto range = std::minmax_element(radii.begin(), radii.end());
            if (*range.first != *range.second) ++tornImages;
            radii.fill(-1);
        }
    }
    // The animation has kept its time: it did not slow down to one source
    // frame per sweep. The last displayed sweep begins at 1.8 seconds.
    if (latchSweeps) assert(seq.displayedFrame() == 72);
    return tornImages;
}
}

int main() {
    std::array<float, 4> phases{};
    for (unsigned spokes : {40U, 80U, 160U, 256U})
        assert(playbackSweepsPerTurn(4, spokes, phases) == 4);
    for (unsigned arms : {1U, 2U, 3U})
        assert(playbackSweepsPerTurn(arms, 160, phases) == 1);
    assert(playbackSweepsPerTurn(4, 159, phases) == 1);
    phases.fill(-17.5f);
    assert(playbackSweepsPerTurn(4, 160, phases) == 4);
    phases[2] += 0.5f;
    assert(playbackSweepsPerTurn(4, 160, phases) == 1);

    PlaybackSweep sweep;
    assert(!sweep.boundary(1, 10000, 500000, 4, 0)); // Starting partway through a sweep.
    assert(!sweep.boundary(1, 124999, 500000, 4, 0));
    assert(sweep.boundary(1, 125000, 500000, 4, 0));
    assert(!sweep.boundary(1, 126000, 510000, 4, 0)); // Period moves phase backward.
    assert(!sweep.boundary(1, 128000, 510000, 4, 0));
    assert(sweep.boundary(1, 500100, 500000, 4, 0)); // Predicted turn wrap.
    assert(!sweep.boundary(2, 0, 500000, 4, 0)); // Real Hall edge: no second latch.
    assert(sweep.boundary(2, 125000, 500000, 4, 0));
    assert(sweep.boundary(2, 490000, 500000, 4, 0));
    assert(sweep.boundary(3, 0, 500000, 4, 0)); // Early Hall edge crosses the wrap.
    sweep.reset();
    assert(!sweep.boundary(3, 0, 500000, 4, -45));
    assert(sweep.boundary(3, 62500, 500000, 4, -45));
    assert(!sweep.boundary(3, 62400, 500000, 4, -45));
    assert(!sweep.boundary(3, 62600, 500000, 4, -45));
    assert(sweep.boundary(3, 187500, 500000, 4, -45));
    assert(sweep.boundary(3, 0, 0, 4, 0)); // No rotation: clock-only presentation.
    assert(!sweep.boundary(4, 70000, 500000, 4, 0)); // Rotation resumes mid-sweep.
    assert(sweep.boundary(4, 125000, 500000, 4, 0));
    sweep.reset();
    assert(!sweep.boundary(1, 0, 500000, 1, 0));
    assert(!sweep.boundary(1, 375000, 500000, 1, 0));
    assert(sweep.boundary(2, 0, 500000, 1, 0));

    const char* path = "build/expanding-ring-fixture.fseq";
    ringFixture(path);
    const unsigned before = simulate(path, false, true, false);
    assert(before == 16); // Reproduce a stepped ring in every complete image.
    assert(simulate(path, true, true, false) == 0);
    assert(simulate(path, true, false, false) == 0);
    assert(simulate(path, true, true, true) == 0); // Slow SD holds an intact ring.
    std::cout << "Expanding-ring regression: " << before
              << " torn images before, zero after; sweep timing and delayed reads passed\n";
}
