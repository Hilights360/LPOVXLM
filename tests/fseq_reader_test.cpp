#include "fseq.hpp"
#include "playback_timing.hpp"
#include <cassert>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#undef fread

namespace {
std::mutex hookMutex;
std::condition_variable hookCondition;
bool blockRead = false, readEntered = false, failRead = false;
int readError = 0;
unsigned transientFailures = 0;
void fixture(const char* name) {
    unsigned char data[44] = {};
    memcpy(data, "PSEQ", 4); data[4] = data[8] = 32; data[7] = 2;
    data[10] = 6; data[14] = 2; data[18] = 25;
    data[32] = 255; data[35] = 255; // Frame zero: two red pixels.
    data[39] = 255; data[42] = 255; // Frame one: two green pixels.
    FILE* f = fopen(name, "wb"); assert(f);
    assert(fwrite(data, 1, sizeof(data), f) == sizeof(data)); fclose(f);
}
}
size_t testSdRead(void* destination, size_t size, size_t count, FILE* file) {
    if (size * count == 6) {
        std::unique_lock<std::mutex> lock(hookMutex);
        if (transientFailures) { --transientFailures; errno = EIO; return 0; }
        if (failRead) { errno = readError; return 0; }
        if (blockRead) {
            readEntered = true; hookCondition.notify_all();
            hookCondition.wait(lock, [] { return !blockRead; });
        }
    }
    return std::fread(destination, size, count, file);
}
int main() {
    using namespace pov;
    const char* path = "build/reader-fixture.fseq";
    fixture(path);
    Fseq seq; std::string error;
    assert(seq.open("/fixture.fseq", path, error));
    assert(seq.channel(0) == 255 && seq.channel(1) == 0);
    assert(seq.channelSpan(0, 6) && seq.channelSpan(0, 6)[3] == 255);
    assert(!seq.channelSpan(0, 0) && !seq.channelSpan(0, 7));
    assert(!seq.channelSpan(5, 2) && !seq.channelSpan(UINT64_MAX, 3));
    seq.ranges = {{10, 3, 0}, {30, 3, 3}};
    assert(seq.channelSpan(10, 3) && seq.channelSpan(30, 3)[0] == 255);
    assert(!seq.channelSpan(11, 3) && !seq.channelSpan(29, 3) && !seq.channelSpan(UINT64_MAX, 3));
    seq.ranges.clear();
    auto job = seq.prepare(1);
    assert(job && seq.loading() && !seq.prepare(0));
    assert(job.run(error));
    assert(seq.channel(0) == 255 && seq.channel(1) == 0); // Still displays old frame.
    assert(seq.publish(job));
    assert(seq.pending() && seq.channel(0) == 255 && seq.channel(1) == 0);
    assert(seq.present() && seq.displayedFrame() == 1);
    assert(seq.channel(0) == 0 && seq.channel(1) == 255);
    assert(seq.channelSpan(0, 6)[1] == 255); // Span follows the latched frame.
    assert(!seq.publish(job)); // Cannot swap the old frame back by republishing.
    assert(!seq.present());

    // Decoding and replacing the pending image never alters the displayed
    // sweep. The latest due frame wins, without queueing obsolete animation.
    job = seq.prepare(0);
    assert(job.run(error) && seq.publish(job));
    assert(seq.channel(1) == 255 && seq.displayedFrame() == 1);
    job = seq.prepare(1);
    assert(job.run(error) && seq.publish(job));
    assert(seq.present() && seq.displayedFrame() == 1 && seq.channel(1) == 255);
    job = seq.prepare(0);
    assert(job.run(error) && seq.publish(job));
    seq.discardPending(); // Pause keeps the displayed image.
    assert(!seq.present() && seq.channel(1) == 255);

    // A card that stalls a single transfer is retried, so the frame arrives.
    transientFailures = 1;
    job = seq.prepare(0);
    assert(job.run(error) && transientFailures == 0);
    assert(seq.publish(job) && seq.present() && seq.channel(0) == 255 && seq.channel(1) == 0);
    job = seq.prepare(1);
    assert(job.run(error) && seq.publish(job) && seq.present() && seq.channel(1) == 255);
    transientFailures = 3; // Two attempts per read, so this one still fails.
    job = seq.prepare(0);
    assert(!job.run(error) && job.ioFailure() && transientFailures == 1);
    transientFailures = 0;
    assert(seq.channel(1) == 255); // An exhausted retry leaves the frame alone.

    failRead = true;
    job = seq.prepare(0);
    assert(!job.run(error) && !seq.publish(job));
    assert(!job.ioFailure()); // A truncated file is not an electrical bus fault.
    assert(seq.channel(1) == 255); // Failed reads cannot expose partial pixels.
    readError = EIO;
    job = seq.prepare(0);
    assert(!job.run(error) && job.ioFailure());
    job = {}; readError = 0;
    failRead = false;

    job = seq.prepare(0);
    assert(job.run(error) && seq.publish(job)); // Red ready for the next sweep.
    blockRead = true; readEntered = false;
    job = seq.prepare(1);
    std::thread worker([&] { assert(job.run(error)); });
    {
        std::unique_lock<std::mutex> lock(hookMutex);
        assert(hookCondition.wait_for(lock, std::chrono::seconds(2), [] { return readEntered; }));
    }
    // Simulate the display noticing an expired load while SD is stuck.
    assert(frameLoadExpired(1000, FrameLoadTimeoutUs + 1000));
    assert(seq.channel(1) == 255);
    assert(seq.present() && seq.channel(0) == 255 && seq.channel(1) == 0);
    const auto before = std::chrono::steady_clock::now();
    seq.close();
    assert(!seq.pending() && !seq.present());
    assert(!seq.channelSpan(0, 3));
    assert(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(50));
    assert(!seq.quiescent()); // Unmount/reopen must not race the SD driver.
    std::string busyError;
    assert(!seq.open("/new.fseq", path, busyError));
    {
        std::lock_guard<std::mutex> lock(hookMutex); blockRead = false;
    }
    hookCondition.notify_all(); worker.join();
    assert(!seq.current(job) && !seq.publish(job)); // Late completion cannot restart playback.
    assert(seq.quiescent());
    assert(seq.open("/new.fseq", path, error));
    auto cancelled = seq.prepare(1);
    seq.close();
    assert(!cancelled.run(error) && !seq.publish(cancelled));
    failRead = true; readError = EIO;
    assert(!seq.open("/io-error.fseq", path, error) && seq.openIoFailure());
    failRead = false; readError = 0;
    assert(!seq.open("/missing.fseq", "build/nonexistent-sd-file.fseq", error) && !seq.openIoFailure());
    assert(!frameLoadExpired(0, FrameLoadTimeoutUs * 4) && !frameLoadExpired(1000, FrameLoadTimeoutUs + 999));

    // A complete non-looping sequence visits its last frame, then stops without
    // ever requesting frame zero again. Single-frame files get one full tick.
    for (uint32_t count : {1U, 2U, 40U, 1200U}) {
        uint32_t frame = 0;
        for (uint32_t tick = 1; tick < count; ++tick) {
            const auto next = advancePlayback(frame, 1, count, false);
            assert(!next.finished && next.frame == tick);
            frame = next.frame;
        }
        assert(!advancePlayback(frame, 0, count, false).finished);
        const auto end = advancePlayback(frame, 1, count, false);
        assert(end.finished && end.frame == count - 1);
        const auto repeat = advancePlayback(frame, 1, count, true);
        assert(!repeat.finished && repeat.frame == 0);
    }
    // Delayed reads may skip intermediate frames, but not the last frame of
    // a single pass. Looping still catches up across multiple complete passes.
    auto last = advancePlayback(30, 100, 40, false);
    assert(!last.finished && last.frame == 39);
    assert(advancePlayback(last.frame, 1, 40, false).finished);
    auto wrapped = advancePlayback(30, 100, 40, true);
    assert(!wrapped.finished && wrapped.frame == 10);
    assert(advancePlayback(wrapped.frame, 29, 40, false).frame == 39);
    assert(advancePlayback(1, UINT32_MAX, 40, true).frame == 16);
    assert(advancePlayback(0, 1, 0, false).finished);

    // At 90 RPM / 80 spokes, 60% duty fits a 1 ms DMA budget; 1% cannot.
    auto gate = spokeGate(0, 8333, 60, false, 3, 80, 1000);
    assert(gate.canStart && gate.nextInUs > 4999 && gate.nextInUs < 5000);
    assert(!spokeGate(0, 8333, 1, false, 3, 80, 1000).fits);
    assert(!spokeGate(0.55, 8333, 60, false, 3, 80, 1000).canStart);
    assert(!spokeGate(0.61, 8333, 60, false, 3, 80, 1000).inside);
    assert(!spokeGate(0, 8333, 0, false, 3, 80, 1000).inside);
    assert(spokeGate(0, 8333, 100, false, 3, 80, 1000).canStart);
    // A late paint after an SD stall must not be emitted in the blank gap.
    assert(!spokeGate(0.9, 8333, 60, false, 3, 80, 1000).canStart);
    assert(!spokeGate(0, 1000, 60, false, 3, 80, 1000).fits);
    assert(!spokeGate(0, 8333, 60, true, 1, 80, 1000).inside);
    assert(spokeGate(0.5, 8333, 60, true, 1, 80, 800).canStart);
    std::cout << "Frame publication, blocked SD cancellation/recovery and spoke deadline tests passed\n";
}
