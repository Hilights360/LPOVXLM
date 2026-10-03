#include "app.hpp"
#include "color_fade.hpp"
#include "playback_timing.hpp"
#include "spoke_patterns.hpp"
#include "alignment_pattern.hpp"
#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <deque>
#include <cstring>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs_flash.h"

namespace pov {
SemaphoreHandle_t stateMutex;
Runtime runtime;
namespace {
SemaphoreHandle_t logsMutex;
std::deque<std::string> logs;
TaskHandle_t displayTaskHandle;
esp_timer_handle_t displayTimer;
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;
HallIndexFilter hallIndex;
RecentTransferPeak autoDutyTransfers;
RecentTransferPeak autoDutyBlankTransfers;
RecentTransferPeak submissionTransfers;

// One paint budget for manual and automatic duty alike. It must cover pixel
// preparation for all arms, DMA packing and transmission, NOT the old
// bit-banged "about a microsecond per wire byte" rule: a 144-pixel APA102 frame
// is 4712 clocks, about 236 us at 20 MHz, where that rule reserved 1271 us.
// Over-reserving makes spokeGate() refuse to paint and reports "spoke timing is
// too short" hundreds of RPM below the real limit. Measured transfers still
// raise the budget, but through the peak's ~2 s decay: a lifetime maximum let a
// single interrupted transfer cap RPM for the rest of the playback session.
uint32_t paintTransferBudget(int64_t now) {
    const uint32_t clocks = SharedClockProtocol::frameBytes(config.pixels,
        static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8;
    const uint32_t wireUs = static_cast<uint32_t>(
        (uint64_t(clocks) * 1000000 + PlaybackDmaClockHz - 1) / PlaybackDmaClockHz);
    // Bootstrap estimate; measured samples include preparation of ALL arms,
    // not just Output::show(). Timer wake-up has a separate duty margin.
    const uint32_t estimate = output.dmaEnabled() ? wireUs + config.pixels * 2 + 150
                                                  : config.pixels * 15 + 250;
    return std::max(estimate, autoDutyTransfers.peak(now)) + 200;
}
uint32_t submissionBudget(int64_t now) {
    const uint32_t clocks = SharedClockProtocol::frameBytes(config.pixels,
        static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8;
    const uint32_t wireUs = static_cast<uint32_t>(
        (uint64_t(clocks) * 1000000 + PlaybackDmaClockHz - 1) / PlaybackDmaClockHz);
    const uint32_t estimate = output.dmaEnabled() ? wireUs + config.pixels * 2 + 50
                                                 : config.pixels * 15 + 150;
    return std::max(estimate, submissionTransfers.peak(now)) + 100;
}
uint32_t automaticBlankBudget(int64_t now, uint32_t paintBudget) {
    // A cached all-black transfer is faster only when every arm's gates line
    // up. Staggered gates must repack the other arms that are still lit.
    const unsigned spokeCount = displaySpokes();
    for (unsigned arm = 1; arm < config.arms; ++arm)
        if (!sameSpokeBoundary(armAngleDegrees(arm, config.armClockwise, config.rotationClockwise) +
            config.armPhase[arm] - config.armPhase[0], spokeCount)) return paintBudget;
    const uint32_t bits = SharedClockProtocol::frameBytes(config.pixels,
        static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8;
    const uint32_t wireUs = (bits + 19) / 20; // Normal playback is 20 MHz.
    return std::max(wireUs + 100, autoDutyBlankTransfers.peak(now)) + 200;
}

void IRAM_ATTR hallIsr(void*) {
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&hallMux);
    const bool accepted = hallIndex.accept(now);
    portEXIT_CRITICAL_ISR(&hallMux);
    if (accepted && displayTaskHandle) {
        BaseType_t wake = pdFALSE;
        vTaskNotifyGiveFromISR(displayTaskHandle, &wake);
        if (wake) portYIELD_FROM_ISR();
    }
}
void displayWake(void*) { notifyDisplay(); }

bool prepareSpokeOutput(std::string& error) {
    if (!output.enableDma(PlaybackDmaClockHz) || !output.show(0)) {
        error = "Cannot start LED DMA: " + std::string(esp_err_to_name(output.lastError()));
        output.disableDma(); runtime.error = error; return false;
    }
    output.stats = {}; // Exclude the LCD driver's first-transfer configuration.
    autoDutyTransfers = {};
    autoDutyBlankTransfers = {};
    submissionTransfers = {};
    runtime.lastFrameLoadUs = runtime.maxFrameLoadUs = runtime.skippedPaints = runtime.missedSpokes = 0;
    runtime.maxBlankStartLateUs = 0;
    runtime.lastPaintPrepareUs = runtime.maxPaintPrepareUs = 0;
    return true;
}

uint64_t channelBase(unsigned arm, unsigned spoke) {
    if (config.autoFseq) {
        const auto derived = fileSettings();
        if (derived.spokes) return fseqImageChannel(config.starts[arm], spoke, 0, derived.channelsPerSpoke);
    }
    const uint64_t oneSpoke = uint64_t(config.arms) * config.pixels * 3;
    const uint64_t extent = sequence.logicalChannels();
    const uint64_t base = config.starts[arm] - 1;
    if (extent == oneSpoke) return base;
    const uint64_t stride = extent % config.spokes == 0 ? extent / config.spokes : oneSpoke;
    return base + uint64_t(spoke) * stride;
}

// Timer notifications wake this task; the callback never transmits LEDs or
// accesses SD. A DMA benchmark temporarily owns the output on this same task.
void displayTask(void*) {
    std::array<int64_t, MaxArms> previousKeys{-1, -1, -1, -1};
    std::array<int64_t, MaxArms> blankDue{};
    uint64_t generation = UINT64_MAX;
    uint64_t playbackSession = UINT64_MAX;
    uint64_t sweepSession = UINT64_MAX;
    PlaybackSweep playbackSweep;
    bool imageSweepStarted = false;
    int64_t lastRender = 0;
    bool wasFlashProof = false;
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t wakeAfter = 20000;
        {
            Lock lock;
            const int64_t now = esp_timer_get_time();
            if (runtime.flashProofUntil && now >= runtime.flashProofUntil) stopFlashProof(true);
            const bool pulse = flashProofRunning();
            if (pulse != wasFlashProof) {
                previousKeys.fill(-1); blankDue.fill(0); wasFlashProof = pulse;
            }
            if (runtime.mode == Mode::ArmOrder && now - runtime.diagnosticStarted >= ArmOrderTestDurationUs)
                stopPlayback();
            if (runtime.mode == Mode::Playback && frameLoadExpired(runtime.frameLoadStarted, now)) {
                ++runtime.sdStalls;
                stopPlayback();
                sequence.close(); // Cancels without waiting for a stalled SD read.
                runtime.error = frameLoadTimeoutReason() + "; lights stopped. Retry SD mount before playback.";
                log("%s", runtime.error.c_str());
                requestSdRecovery(frameLoadTimeoutReason());
            }
            if (runtime.mode == Mode::StripSpeed) {
                stepStripSpeedTest();
                wakeAfter = 1000; // Yield between sustained transfers; keep controls responsive.
            } else if (runtime.mode == Mode::DmaTest) {
                stepDmaTest();
                wakeAfter = 50000;
            } else if (runtime.mode == Mode::SignalCheck) {
                const int64_t elapsed = now - runtime.diagnosticStarted;
                const bool high = runtime.signalPattern == 2 ? (elapsed / 500000) % 2 != 0 : runtime.signalPattern == 1;
                if (generation != runtime.generation || high != runtime.signalHigh) {
                    if (!output.signalLevel(runtime.signalPin, high)) {
                        runtime.error = "Signal check failed: " + std::string(esp_err_to_name(output.lastError()));
                        stopPlayback();
                    } else runtime.signalHigh = high;
                    generation = runtime.generation;
                }
                lastRender = 0;
                wakeAfter = runtime.signalPattern == 2 ? 500000 - elapsed % 500000 : 100000;
            } else {
                const HallSnapshot hall = hallSnapshot();
                const bool turning = hall.period > 0 && now - hall.last < std::max<int64_t>(1000000, hall.period * 3);
                if (sweepSession != runtime.playbackSession) {
                    playbackSweep.reset();
                    imageSweepStarted = false;
                    sweepSession = runtime.playbackSession;
                }
                if (runtime.mode == Mode::Playback) {
                    const bool boundary = playbackSweep.boundary(hall.count, now - hall.last,
                        turning ? hall.period : 0,
                        playbackSweepsPerTurn(config.arms, displaySpokes(), config.armPhase),
                        config.phase + config.armPhase[0]);
                    if (boundary && !runtime.paused) {
                        if (sequence.present()) {
                            runtime.frame = sequence.displayedFrame();
                            ++runtime.generation;
                        } else if (imageSweepStarted && !config.loop && runtime.playbackFinishDue && now >= runtime.playbackFinishDue) {
                            // The final frame has now had a complete sweep, even
                            // when the file's frame interval is much shorter.
                            log("Playback finished: %s", sequence.path.c_str());
                            stopPlayback();
                            runtime.playbackComplete = true;
                        }
                        imageSweepStarted = true;
                    }
                }
                const bool angular = usesSpokeTiming(runtime.mode);
                const unsigned spokeCount = displaySpokes();
                if (playbackSession != runtime.playbackSession) lastRender = 0;
                std::array<int64_t, MaxArms> keys{-1, -1, -1, -1};
                std::array<unsigned, MaxArms> spokes{};
                std::array<std::array<uint8_t, 3>, MaxArms> alignmentColors{};
                std::array<int64_t, MaxArms> paintDeadlines{};
                const bool automatic = config.autoDuty && usesDisplayDuty(runtime.mode) && !pulse;
                const uint32_t pulseWireUs = (SharedClockProtocol::frameBytes(config.pixels,
                    static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8 + 19) / 20;
                // Pulses get a final deadline check after their actual packing
                // work. Do not reject them using the old normal-playback pack
                // estimate before the faster pulse buffer has been prepared.
                const int64_t transferBudget = pulse
                    ? pulseWireUs * (runtime.flashProofColorFrames + 1) + 100
                    : paintTransferBudget(now);
                const uint32_t submitBudget = submissionBudget(now);
                const auto calculated = calculateDuty(turning ? hall.period : 0, spokeCount, transferBudget,
                    automatic ? automaticBlankBudget(now, transferBudget) : transferBudget);
                const unsigned duty = pulse ? 100 : automatic ? calculated.percent : config.duty;
                runtime.timingLimited = false;
                unsigned testArm = 0, testPixel = 0, testColor = 0;
                std::array<uint8_t, 3> fade{};
                if (runtime.mode == Mode::Arms) {
                    const unsigned stepMs = std::max(15U, 600U / config.pixels);
                    const uint64_t step = (now - runtime.lastActivity) / (stepMs * 1000ULL);
                    testPixel = step % config.pixels;
                    testArm = (step / config.pixels) % config.arms;
                    testColor = (step / config.pixels / config.arms) % 3;
                    keys[testArm] = testPixel + testColor * MaxPixels;
                    wakeAfter = stepMs * 1000;
                } else if (runtime.mode == Mode::ArmOrder) {
                    const int arm = armOrderTestArm(now - runtime.diagnosticStarted, config.arms);
                    if (arm >= 0) keys[arm] = arm;
                    wakeAfter = 50000;
                } else if (runtime.mode == Mode::Connectors) {
                    for (unsigned arm = 0; arm < config.arms; ++arm) keys[arm] = arm;
                } else if (runtime.mode == Mode::ColorFade) {
                    constexpr int64_t frameUs = ColorFadeFrameMs * 1000;
                    const int64_t elapsed = now - runtime.diagnosticStarted;
                    const int64_t frame = elapsed / frameUs;
                    fade = colorFadeAt(frame * ColorFadeFrameMs);
                    for (unsigned arm = 0; arm < config.arms; ++arm) keys[arm] = frame;
                    wakeAfter = frameUs - elapsed % frameUs;
                } else if (runtime.mode == Mode::WhiteBlink) {
                    const int64_t elapsed = now - runtime.diagnosticStarted;
                    const int64_t halfCycle = elapsed / 500000;
                    if (halfCycle % 2 == 0)
                        for (unsigned arm = 0; arm < config.arms; ++arm) keys[arm] = halfCycle;
                    wakeAfter = 500000 - elapsed % 500000;
                } else if (runtime.mode == Mode::Solid) {
                    const int64_t elapsed = now - runtime.diagnosticStarted;
                    for (unsigned arm = 0; arm < config.arms; ++arm) keys[arm] = elapsed / 100000;
                    wakeAfter = 100000 - elapsed % 100000;
                } else if (runtime.mode == Mode::Hall) {
                    if (!gpio_get_level(static_cast<gpio_num_t>(BoardPins::Hall)))
                        for (unsigned arm = 0; arm < config.arms; ++arm) keys[arm] = 1;
                    wakeAfter = 2000;
                } else if (angular && turning && (runtime.mode != Mode::Playback || !sequence.path.empty())) {
                    const double period = hall.period;
                    const double spokeUs = period / spokeCount;
                    if (runtime.mode != Mode::Alignment && lastRender && now - lastRender > spokeUs * 2)
                        runtime.missedSpokes += static_cast<uint32_t>((now - lastRender) / spokeUs) - 1;
                    lastRender = now;
                    runtime.spoke = static_cast<unsigned>((now - hall.last) / spokeUs) % spokeCount;
                    for (unsigned arm = 0; arm < config.arms; ++arm) {
                        const double phase = config.phase + armAngleDegrees(arm, config.armClockwise,
                            config.rotationClockwise) + config.armPhase[arm];
                        if (runtime.mode == Mode::Alignment) {
                            const double angle = spokePosition(now - hall.last, hall.period, 360, phase);
                            const auto segment = alignmentSegment(angle);
                            const double duration = (segment.end - segment.start) * period / 360;
                            const auto gate = spokeGate((angle - segment.start) / (segment.end - segment.start),
                                duration, 100, false, 0, 1, transferBudget);
                            const bool lit = segment.color[0] || segment.color[1];
                            runtime.timingLimited |= lit && !gate.fits;
                            const int64_t key = int64_t(hall.count) * 8 + segment.index;
                            if (lit && gate.fits) {
                                if (gate.canStart || (previousKeys[arm] == key && playbackSession == runtime.playbackSession)) {
                                    keys[arm] = key;
                                    alignmentColors[arm] = segment.color;
                                    paintDeadlines[arm] = now + static_cast<int64_t>(gate.closeInUs);
                                } else ++runtime.skippedPaints;
                            }
                            wakeAfter = std::min<int64_t>(wakeAfter, static_cast<int64_t>(gate.nextInUs));
                            continue;
                        }
                        const double position = spokePosition(now - hall.last, hall.period, spokeCount,
                            phase);
                        const double fraction = position - std::floor(position);
                        spokes[arm] = static_cast<unsigned>(position) % spokeCount;
                        const auto gate = spokeGate(fraction, spokeUs, duty, config.strobe && !automatic && !pulse,
                                                    config.strobeWidth, spokeCount, transferBudget, calculated.blankUs);
                        runtime.timingLimited |= automatic ? !calculated.feasible : duty && !gate.fits;
                        const int64_t key = int64_t(hall.count) * spokeCount + spokes[arm];
                        if (gate.inside && gate.fits) {
                            // An already-painted spoke may remain until its blank boundary;
                            // a new/late paint must finish before that boundary.
                            if (gate.canStart || (previousKeys[arm] == key && playbackSession == runtime.playbackSession)) {
                                keys[arm] = key;
                                paintDeadlines[arm] = now + static_cast<int64_t>(gate.closeInUs);
                            } else ++runtime.skippedPaints;
                        }
                        wakeAfter = std::min<int64_t>(wakeAfter, static_cast<int64_t>(gate.nextInUs));
                    }
                } else lastRender = 0;
                // The image above is latched for a whole sweep. Rendering still
                // follows spoke/blank boundaries, never a decoder notification.
                const bool changed = angular ? playbackSession != runtime.playbackSession : generation != runtime.generation;
                if (changed || keys != previousKeys) {
                    const bool preparingPaint = angular && !pulse && std::any_of(keys.begin(), keys.end(),
                        [](int64_t key) { return key >= 0; });
                    output.clear();
                    for (unsigned arm = 0; arm < config.arms; ++arm) {
                        if (keys[arm] >= 0 && angular && !pulse &&
                            !remainingPaintFits(now, esp_timer_get_time(), paintDeadlines[arm], transferBudget, submitBudget)) {
                            keys[arm] = -1; ++runtime.skippedPaints;
                        }
                        if (keys[arm] < 0) continue;
                        const uint64_t base = runtime.mode == Mode::Playback ? channelBase(arm, spokes[arm]) : 0;
                        if (runtime.mode == Mode::Playback) {
                            if (const auto* colors = sequence.channelSpan(base, config.pixels * 3)) {
                                output.row(arm, colors, runtime.sequenceTipFirst);
                                continue;
                            }
                        }
                        const auto patternColor = spokeColor(runtime.mode == Mode::QuarterColors
                            ? SpokePattern::Quarters : SpokePattern::Alternating, spokes[arm], spokeCount);
                        for (unsigned pixel = 0; pixel < config.pixels; ++pixel) {
                            uint8_t r = 0, g = 0, b = 0;
                            switch (runtime.mode) {
                            case Mode::Playback:
                                r = sequence.channel(base + pixel * 3);
                                g = sequence.channel(base + pixel * 3 + 1);
                                b = sequence.channel(base + pixel * 3 + 2);
                                break;
                            case Mode::Hall: r = 255; break;
                            case Mode::WhiteBlink: r = g = b = 255; break;
                            case Mode::ColorFade:
                                r = fade[0]; g = fade[1]; b = fade[2];
                                break;
                            case Mode::Solid:
                                r = runtime.testColor[0]; g = runtime.testColor[1]; b = runtime.testColor[2];
                                break;
                            case Mode::QuarterColors: case Mode::AlternatingSpokes:
                                r = patternColor[0]; g = patternColor[1]; b = patternColor[2];
                                break;
                            case Mode::Alignment:
                                r = alignmentColors[arm][0]; g = alignmentColors[arm][1]; b = alignmentColors[arm][2];
                                break;
                            case Mode::Connectors:
                                r = (arm == 0 || arm == 3) ? 255 : 0;
                                g = (arm == 1 || arm == 3) ? 255 : 0;
                                b = (arm == 2 || arm == 3) ? 255 : 0;
                                break;
                            case Mode::Arms:
                                if (pixel == testPixel) {
                                    r = testColor == 0 ? 255 : 0;
                                    g = testColor == 1 ? 255 : 0;
                                    b = testColor == 2 ? 255 : 0;
                                }
                                break;
                            case Mode::ArmOrder:
                                r = 255; g = b = arm == 0 ? 0 : 255;
                                break;
                            case Mode::Stopped: case Mode::DmaTest: case Mode::SignalCheck: case Mode::StripSpeed: break;
                            }
                            output.pixel(arm, pixel, r, g, b, runtime.mode != Mode::ArmOrder,
                                runtime.mode == Mode::Playback && runtime.sequenceTipFirst);
                        }
                    }
                    if (preparingPaint) {
                        const int64_t prepared = esp_timer_get_time();
                        runtime.lastPaintPrepareUs = static_cast<uint32_t>(prepared - now);
                        runtime.maxPaintPrepareUs = std::max(runtime.maxPaintPrepareUs, runtime.lastPaintPrepareUs);
                        // Check the remaining packing/transmission once all
                        // arms are ready. A rejected paint must still teach the
                        // next calculation how long preparation really took.
                        const bool late = std::any_of(paintDeadlines.begin(), paintDeadlines.begin() + config.arms,
                            [&](int64_t deadline) { return deadline && prepared + submitBudget > deadline; });
                        if (late) {
                            autoDutyTransfers.observe(prepared, runtime.lastPaintPrepareUs + submitBudget);
                            for (auto key : keys) if (key >= 0) ++runtime.skippedPaints;
                            keys.fill(-1); output.clear();
                        }
                    }
                    if (angular && !pulse) {
                        const int64_t submit = esp_timer_get_time();
                        for (unsigned arm = 0; arm < config.arms; ++arm) {
                            if (playbackSession == runtime.playbackSession && previousKeys[arm] >= 0 &&
                                keys[arm] != previousKeys[arm] && blankDue[arm] > 0)
                                runtime.maxBlankStartLateUs = std::max<uint32_t>(runtime.maxBlankStartLateUs,
                                    static_cast<uint32_t>(std::max<int64_t>(0, submit - blankDue[arm])));
                            blankDue[arm] = keys[arm] >= 0 ? paintDeadlines[arm] : 0;
                        }
                    }
                    const unsigned brightness = runtime.mode == Mode::ArmOrder
                        ? std::min<unsigned>(config.brightness, 10) : config.brightness;
                    int64_t pulseDeadline = 0;
                    if (pulse) for (unsigned arm = 0; arm < config.arms; ++arm)
                        if (keys[arm] >= 0 && (!pulseDeadline || paintDeadlines[arm] < pulseDeadline)) pulseDeadline = paintDeadlines[arm];
                    if (!output.show(brightness * 255U / 100U, pulse, pulseDeadline)) {
                        const std::string error = "LED output failed: " + std::string(esp_err_to_name(output.lastError()));
                        if (runtime.error != error) log("%s", error.c_str());
                        runtime.error = error;
                        if (pulse) stopFlashProof();
                    } else if (runtime.mode == Mode::WhiteBlink) {
                        const bool lit = keys[0] >= 0;
                        if (lit != runtime.whiteBlinkOn) ++runtime.whiteBlinkTransitions;
                        runtime.whiteBlinkOn = lit;
                    }
                    if (pulse) {
                        if (output.stats.lastSkipped) { ++runtime.flashProofSkipped; runtime.skippedPaints += config.arms; }
                        else if (output.stats.lastPulse) ++runtime.flashProofBursts;
                    }
                    if (angular && !pulse && output.dmaEnabled() && output.lastError() == ESP_OK) {
                        const bool allBlack = !config.brightness ||
                            std::all_of(keys.begin(), keys.end(), [](int64_t key) { return key < 0; });
                        const int64_t completed = esp_timer_get_time();
                        if (allBlack) autoDutyBlankTransfers.observe(completed, output.stats.lastUs);
                        else {
                            autoDutyTransfers.observe(completed, static_cast<uint32_t>(completed - now));
                            submissionTransfers.observe(completed, output.stats.lastUs);
                        }
                    }
                    previousKeys = keys; generation = runtime.generation;
                    playbackSession = runtime.playbackSession;
                }
                // Subtract the work just performed from the desired next boundary.
                wakeAfter = std::max<int64_t>(100, wakeAfter - (esp_timer_get_time() - now));
            }
        }
        esp_timer_stop(displayTimer);
        esp_timer_start_once(displayTimer, wakeAfter);
    }
}

void playbackTask(void*) {
    // Decode ahead on the animation clock. Publishing replaces only the
    // pending image; the display task adopts it at a complete sweep boundary.
    Fseq::ReadJob staged;
    int64_t stagedDue = 0;
    uint64_t timingSession = UINT64_MAX;
    uint32_t decodedFrame = 0;
    unsigned failures = 0;
    bool watchdog = false;
    while (true) {
        bool load = false;
        uint64_t session = 0;
        {
            Lock lock;
            sequence.quiescent(); // Reap cancelled reads after the SD driver returns.
            serviceSdRecovery();
            if (config.watchdog != watchdog) {
                if (config.watchdog) {
                    esp_task_wdt_config_t cfg = {};
                    cfg.timeout_ms = 8000;
                    cfg.trigger_panic = true;
                    esp_err_t err = esp_task_wdt_init(&cfg);
                    if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_reconfigure(&cfg);
                    if (err == ESP_OK) err = esp_task_wdt_add(nullptr);
                    watchdog = err == ESP_OK;
                    if (!watchdog) { log("Watchdog start failed: %s", esp_err_to_name(err)); config.watchdog = false; }
                } else {
                    esp_task_wdt_delete(nullptr); esp_task_wdt_deinit(); watchdog = false;
                }
            }
            const int64_t now = esp_timer_get_time();
            if (timingSession != runtime.playbackSession) {
                staged = {}; stagedDue = 0; failures = 0;
                timingSession = runtime.playbackSession;
                decodedFrame = runtime.frame;
            }
            if (runtime.mode == Mode::Playback && !runtime.paused && !sequence.path.empty()) {
                const int64_t frameUs = sequenceFrameUs();
                auto schedule = [&](uint32_t advance, int64_t due) {
                    const auto next = advancePlayback(decodedFrame, advance, sequence.header.frames, config.loop);
                    if (next.finished) {
                        runtime.playbackFinishDue = due;
                        stagedDue = 0;
                    } else {
                        runtime.playbackFinishDue = 0;
                        staged = sequence.prepare(next.frame);
                        stagedDue = due;
                    }
                };
                if (staged && staged.complete() && now >= stagedDue) {
                    if (sequence.publish(staged)) {
                        decodedFrame = staged.frame(); ++runtime.framesRead;
                    }
                    // Falling behind still skips frames, but the read for the
                    // frame after this one starts now rather than when it is due.
                    const uint32_t advance = 1 + static_cast<uint32_t>((now - stagedDue) / frameUs);
                    staged = {};
                    schedule(advance, stagedDue + int64_t(advance) * frameUs);
                } else if (!staged && (!runtime.playbackFinishDue || config.loop)) {
                    schedule(1, now + frameUs); // Prime, or recover a dropped read.
                }
                if (staged && !staged.complete()) {
                    load = true;
                    session = runtime.playbackSession;
                    runtime.frameLoadStarted = now;
                }
            } else {
                staged = {}; stagedDue = 0;
                if (runtime.paused) {
                    // Resume from the image actually on the rotor, not a
                    // newer pending image that was never displayed.
                    sequence.discardPending();
                    decodedFrame = runtime.frame;
                    runtime.playbackFinishDue = 0;
                }
                if (!runtime.maintenance && !runtime.playbackComplete && runtime.mode == Mode::Stopped && now - runtime.lastActivity >= 5000000) {
                    std::string path;
                    if (config.background && !config.backgroundPath.empty()) path = config.backgroundPath;
                    else if (config.autoplay && now - runtime.lastActivity >= 300000000) path = "/test2.fseq";
                    if (!path.empty()) {
                        std::string error;
                        if (!startPlayback(path, error)) { runtime.error = error; runtime.lastActivity = now; }
                    }
                }
            }
            if (watchdog) esp_task_wdt_reset();
        }
        if (load) {
            const int64_t started = esp_timer_get_time();
            std::string error;
            // Neither SD reads nor decompression can hold up LED blanking.
            const bool loaded = staged.run(error);
            const uint32_t elapsed = static_cast<uint32_t>(esp_timer_get_time() - started);
            const bool ioFailure = staged.ioFailure();
            Lock lock;
            if (sequence.current(staged) && session == runtime.playbackSession && runtime.mode == Mode::Playback) {
                runtime.lastFrameLoadUs = elapsed;
                runtime.maxFrameLoadUs = std::max(runtime.maxFrameLoadUs, elapsed);
                runtime.frameLoadStarted = 0;
                if (elapsed >= FrameLoadTimeoutUs) {
                    ++runtime.sdStalls;
                    stopPlayback(); sequence.close();
                    staged = {};
                    runtime.error = frameLoadTimeoutReason() + "; lights stopped. Retry SD mount before playback.";
                    log("%s", runtime.error.c_str());
                    requestSdRecovery(frameLoadTimeoutReason());
                } else if (loaded) {
                    failures = 0; // Published at stagedDue, not on arrival.
                } else {
                    staged = {}; // Read this frame again on the next pass.
                    runtime.error = error;
                    log("Playback: %s", error.c_str());
                    if (++failures >= 3) {
                        stopPlayback(); sequence.close();
                        runtime.error = "Repeated SD read failures; lights stopped. Retry SD mount before playback.";
                        if (ioFailure) requestSdRecovery("Repeated SD read failures");
                    }
                }
            } else { staged = {}; stagedDue = 0; failures = 0; }
            if (watchdog) esp_task_wdt_reset();
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
}

void log(const char* format, ...) {
    char text[384];
    va_list args; va_start(args, format); vsnprintf(text, sizeof(text), format, args); va_end(args);
    ESP_LOGI("POV", "%s", text);
    if (!logsMutex) return;
    xSemaphoreTake(logsMutex, portMAX_DELAY);
    logs.emplace_back(text);
    if (logs.size() > 200) logs.pop_front();
    xSemaphoreGive(logsMutex);
}
std::string logText() {
    xSemaphoreTake(logsMutex, portMAX_DELAY);
    std::string result;
    for (const auto& line : logs) { result += line; result += '\n'; }
    xSemaphoreGive(logsMutex);
    return result;
}
void clearLogs() { xSemaphoreTake(logsMutex, portMAX_DELAY); logs.clear(); xSemaphoreGive(logsMutex); }
HallSnapshot hallSnapshot() {
    portENTER_CRITICAL(&hallMux);
    const HallIndexFilter value = hallIndex;
    portEXIT_CRITICAL(&hallMux);
    return {value.last, value.averagePeriod(), value.lastPeriod,
        value.lastRejectedInterval, value.count, value.rejected, value.samples};
}
float rpm() {
    const auto hall = hallSnapshot();
    if (!hall.period || esp_timer_get_time() - hall.last > std::max<int64_t>(1000000, hall.period * 3)) return 0;
    return 60000000.0 / hall.period;
}
DutyCalculation dutyCalculation() {
    const auto hall = hallSnapshot();
    const int64_t now = esp_timer_get_time();
    const bool turning = hall.period > 0 && now - hall.last < std::max<int64_t>(1000000, hall.period * 3);
    const uint32_t paintBudget = paintTransferBudget(now);
    return calculateDuty(turning ? hall.period : 0, displaySpokes(), paintBudget, automaticBlankBudget(now, paintBudget));
}
FseqSettings fileSettings() {
    return deriveFseqSettings(runtime.fileHeader, runtime.fileRanges, config.pixels, config.arms, config.starts);
}
unsigned displaySpokes() {
    if (!config.autoFseq) return config.spokes;
    const auto derived = fileSettings();
    return derived.spokes ? derived.spokes : config.spokes;
}
int64_t sequenceFrameUs() {
    if (!config.autoFseq) return 1000000 / config.fps;
    const auto derived = fileSettings();
    return derived.frameUs ? derived.frameUs : 1000000 / config.fps;
}
void notifyDisplay() { if (displayTaskHandle) xTaskNotifyGive(displayTaskHandle); }
void stopPlayback() {
    stopFlashProof();
    cancelStripSpeedTest();
    cancelDmaTest();
    if (runtime.mode == Mode::SignalCheck) output.disableDma(); // Restore all signal pins LOW before blanking.
    runtime.mode = Mode::Stopped; runtime.paused = false; runtime.background = false;
    runtime.whiteBlinkOn = false;
    runtime.playbackComplete = false;
    ++runtime.playbackSession;
    runtime.frameLoadStarted = 0;
    runtime.playbackFinishDue = 0;
    runtime.timingLimited = false;
    autoDutyTransfers = {};
    autoDutyBlankTransfers = {};
    submissionTransfers = {};
    runtime.lastActivity = esp_timer_get_time();
    ++runtime.generation;
    output.clear(); output.show(0); notifyDisplay();
    if (output.dmaEnabled()) output.disableDma();
}
bool startPlayback(const std::string& path, std::string& error) {
    if (runtime.maintenance) { error = "Storage maintenance is running"; return false; }
    stopPlayback();
    runtime.fileHeader = {}; runtime.fileRanges.clear(); runtime.filePath.clear();
    if (!output.ready()) { error = "LED output unavailable"; return false; }
    std::string native;
    if (!sd.ready || !sdPath(path, native)) { error = "SD unavailable or invalid path"; runtime.error = error; return false; }
    if (!sequence.open(path, native, error)) {
        runtime.error = error;
        if (sequence.openIoFailure()) requestSdRecovery("SD sequence open/read failed");
        return false;
    }
    if (!prepareSpokeOutput(error)) {
        sequence.close(); return false;
    }
    runtime.fileHeader = sequence.header;
    runtime.fileRanges = sequence.ranges;
    runtime.filePath = path;
    runtime.sequenceTipFirst = loadSequenceTipFirst(path);
    runtime.mode = Mode::Playback; runtime.frame = 0; runtime.framesRead = 1;
    runtime.background = path.rfind("/BGEffects/", 0) == 0;
    runtime.error.clear(); ++runtime.generation;
    log("FSEQ: %s, %lu frames, %lu channels, compression %u", path.c_str(),
        static_cast<unsigned long>(sequence.header.frames), static_cast<unsigned long>(sequence.header.channels), sequence.header.compression);
    notifyDisplay(); return true;
}
void startDiagnostic(Mode mode) {
    stopPlayback();
    runtime.diagnosticStarted = esp_timer_get_time();
    if (mode == Mode::WhiteBlink) runtime.whiteBlinkTransitions = 0;
    runtime.mode = mode; ++runtime.generation; notifyDisplay();
}
bool startSpokeTest(Mode mode, std::string& error) {
    if (mode != Mode::QuarterColors && mode != Mode::AlternatingSpokes && mode != Mode::Alignment) {
        error = "Choose quarter colors, alternating spokes or arm alignment"; return false;
    }
    if (!output.ready()) { error = "LED output unavailable"; return false; }
    stopPlayback();
    sequence.close(); // Cancel pending playback without waiting for SD.
    if (!prepareSpokeOutput(error)) return false;
    runtime.frame = runtime.framesRead = 0;
    runtime.diagnosticStarted = esp_timer_get_time();
    runtime.mode = mode; runtime.error.clear(); ++runtime.generation;
    if (mode == Mode::Alignment)
        log("Arm alignment: green half, three red rays, %u arms, %.1f degree offset", unsigned(config.arms), double(config.phase));
    else
        log("LED spoke test: %s, %u spokes, %u arms, %u%% duty",
            mode == Mode::QuarterColors ? "quarters" : "alternating", displaySpokes(),
            unsigned(config.arms), unsigned(config.duty));
    notifyDisplay(); return true;
}
bool flashProofRunning() {
    return runtime.mode == Mode::Playback && runtime.flashProofUntil > esp_timer_get_time();
}
bool startFlashProof(std::string& error, unsigned colorFrames) {
    if (!colorFrames) colorFrames = runtime.flashProofColorFrames;
    if (colorFrames > MaximumPulseColorFrames) { error = "Choose a flash width from 1 to 3"; return false; }
    if (runtime.mode != Mode::Playback || runtime.paused || !output.dmaEnabled()) {
        error = "Start and resume a sequence before the flash test"; return false;
    }
    if (config.arms != 4 || config.pixels != 144 || config.ledProtocol != 1) {
        error = "This verification uses four 144-pixel APA102 outputs"; return false;
    }
    if (!config.brightness) { error = "Set brightness above zero before the flash test"; return false; }
    for (unsigned arm = 1; arm < config.arms; ++arm)
        if (!sameSpokeBoundary(armAngleDegrees(arm, config.armClockwise, config.rotationClockwise) +
            config.armPhase[arm] - config.armPhase[0], displaySpokes())) {
            error = "The flash test requires synchronized arm spoke boundaries"; return false;
        }
    if (!output.preparePulse(colorFrames)) { error = "Cannot allocate the flash test buffer"; return false; }
    output.clear();
    if (!output.show(0)) { error = "Cannot blank before the flash test"; return false; }
    runtime.flashProofUntil = esp_timer_get_time() + 30000000;
    runtime.flashProofColorFrames = colorFrames;
    runtime.flashProofBursts = runtime.flashProofSkipped = 0;
    runtime.flashProofState = "running";
    ++runtime.generation; notifyDisplay();
    log("30-second flash verification: four 144-pixel outputs, %u color frames + black in one DMA transaction", colorFrames);
    return true;
}
void stopFlashProof(bool complete) {
    if (!runtime.flashProofUntil) return;
    runtime.flashProofUntil = 0;
    runtime.flashProofState = complete ? "complete" : "cancelled";
    runtime.timingLimited = false;
    ++runtime.generation; notifyDisplay();
    log("Flash verification %s: %lu bursts, %lu late prepared pulses skipped", runtime.flashProofState.c_str(),
        static_cast<unsigned long>(runtime.flashProofBursts), static_cast<unsigned long>(runtime.flashProofSkipped));
}
bool reconfigureOutput() {
    stopPlayback();
    if (!output.begin(config.pixels)) { runtime.error = "LED buffer allocation failed"; return false; }
    ++runtime.generation; notifyDisplay(); return true;
}
void restartSoon() {
    xTaskCreate([](void*) { vTaskDelay(pdMS_TO_TICKS(750)); esp_restart(); }, "restart", 2048, nullptr, 1, nullptr);
}
}

extern "C" void app_main() {
    using namespace pov;
    stateMutex = xSemaphoreCreateRecursiveMutex();
    logsMutex = xSemaphoreCreateMutex();
    configASSERT(stateMutex && logsMutex);
    log("LPOVXLM native ESP-IDF: four outputs, one Hall pulse per revolution");
    log("Boot reset reason: %d", static_cast<int>(esp_reset_reason()));
    log("PSRAM: %u bytes; free %u bytes", static_cast<unsigned>(esp_psram_get_size()),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    // Preserve the existing 'display' namespace; never erase settings on init failure.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(loadSettings());
    if (mountSd()) {
        restoreSettingsBackup();
        installSdFirmware();
    }
    ESP_ERROR_CHECK(saveSettings());
    saveSettingsBackup();
    if (!output.begin(config.pixels)) log("LED output allocation failed");
    runtime.lastActivity = esp_timer_get_time();
    gpio_config_t hall = {};
    hall.pin_bit_mask = 1ULL << BoardPins::Hall;
    hall.mode = GPIO_MODE_INPUT;
    hall.pull_up_en = GPIO_PULLUP_ENABLE;
    hall.intr_type = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&hall));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(static_cast<gpio_num_t>(BoardPins::Hall), hallIsr, nullptr));
    esp_timer_create_args_t timer = {};
    timer.callback = displayWake; timer.name = "display";
    ESP_ERROR_CHECK(esp_timer_create(&timer, &displayTimer));
    configASSERT(xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 3, &displayTaskHandle, 1) == pdPASS);
    configASSERT(xTaskCreatePinnedToCore(playbackTask, "playback", 8192, nullptr, 2, nullptr, 0) == pdPASS);
    startNetwork();
    startWeb();
    notifyDisplay();
    log("Ready: http://%s.local/ or http://192.168.4.1/ ; UART monitor at 115200 baud",
        config.hostname.c_str());
}
