#include "app.hpp"
#include "sd_benchmark.hpp"
#include "sd_recovery.hpp"
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <sys/stat.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"

namespace pov {
namespace {
constexpr size_t BlockBytes = 16 * 1024;
std::atomic<bool> running{false}, cancel{false};
struct SdToolState {
    bool format = false, mount = false, readOnly = false, scope = false;
    bool fallback = true;
    const char* state = "idle";
    uint32_t bytes = 0;
    unsigned width = 0, frequency = 0, mode = 0, maxFrequency = 0;
    int64_t started = 0, elapsed = 0;
    SdBenchmarkResult benchmark;
    std::string error, temporaryFile;
    SdReadTestOptions readOptions;
    uint32_t blinkTransitionsStart = 0, blinkTransitions = 0;
    bool blinkInterrupted = false;
    SdTimingOptions timing;
    std::string sha256;
    SdScopeOptions scopeOptions;
    SdScopeResult scopeResult;
    std::vector<SdScopeResult> scopeChannels;
    unsigned scopeCompleted = 0;
    int scopeActivePin = SdScopeAllPins;
    SdState beforeScope;
    std::string restoreError;
    bool wifiOffDuringCapture = false, wifiRestored = false;
    int64_t wifiStoppedOffsetUs = 0, wifiResumedOffsetUs = 0;
    std::string wifiError;
} job;
struct RecoveryState {
    bool active = false, pending = false;
    const char* state = "idle";
    unsigned mode = 0, maximum = 0, attempts = 0, successes = 0;
    unsigned width = 0, frequency = 0;
    int64_t started = 0, elapsed = 0;
    SdState failed;
    std::string reason, error;
} recovery;

void recoveryWorker(void*) {
    {
    SdState mounted;
    std::string error;
    const bool ok = recoverSdCard(recovery.mode, recovery.maximum, recovery.failed, mounted, error);
    {
        Lock lock;
        sd = mounted;
        recovery.error = error; recovery.state = ok ? "complete" : "failed";
        recovery.elapsed = esp_timer_get_time() - recovery.started;
        if (ok) {
            ++recovery.successes;
            runtime.error = "SD remounted at " + std::to_string(sd.width) + "-bit/" +
                std::to_string(sd.frequency) + " kHz; run an SD speed test to verify this setting.";
        } else runtime.error = error;
        log("%s", runtime.error.c_str());
        recovery.active = false; runtime.maintenance = false;
        runtime.lastActivity = esp_timer_get_time(); running = false;
    }
    } // Destroy strings before deleting this FreeRTOS task.
    vTaskDelete(nullptr);
}

struct Hooks {
    int64_t now() { return esp_timer_get_time(); }
    bool cancelled() { return cancel.load(); }
    bool sync(FILE* file) { return fflush(file) == 0 && fsync(fileno(file)) == 0; }
    void progress(const SdBenchmarkResult& value) {
        { Lock lock; job.benchmark = value; }
        vTaskDelay(1); // Keep networking and idle tasks scheduled between blocks.
    }
};
struct ReadHooks : Hooks {
    mbedtls_sha256_context* digest = nullptr;
    bool consume(const uint8_t* data, size_t size) { return mbedtls_sha256_update(digest, data, size) == 0; }
};
bool readTest(SdBenchmarkResult& result, std::string& error) {
    result.phase = "reading";
    const auto& options = job.readOptions;
    std::string native;
    if (!sdPath(options.path, native)) { error = "Invalid SD read path"; return false; }
    FILE* file = fopen(native.c_str(), "rb");
    if (!file) { error = sdFileError("Cannot open SD read test file", errno); return false; }
    struct stat info = {};
    if (fstat(fileno(file), &info) != 0 || !S_ISREG(info.st_mode) || info.st_size < job.bytes) {
        fclose(file); error = "Read test needs an existing file at least as large as the requested size"; return false;
    }
    auto* buffer = static_cast<uint8_t*>(heap_caps_malloc(options.blockBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    auto* stream = static_cast<char*>(heap_caps_malloc(options.blockBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    mbedtls_sha256_context digest;
    mbedtls_sha256_init(&digest);
    uint8_t hash[32] = {};
    bool ok = false, timingChanged = false;
    const auto oldLogLevel = esp_log_level_get("sdmmc_req");
    if (!buffer || !stream) error = "Not enough internal DMA memory for SD read test";
    else if (setvbuf(file, stream, _IOFBF, options.blockBytes) != 0) error = "Cannot configure SD read buffer";
    else if (mbedtls_sha256_starts(&digest, 0) != 0) error = "Cannot initialize SD read digest";
    else {
        timingChanged = true;
        const esp_err_t err = configureSdReadTiming(options.delayPhase, options.continuousClock);
        if (err != ESP_OK) error = "Cannot set SD diagnostic timing: " + std::string(esp_err_to_name(err));
        else {
            esp_log_level_set("sdmmc_req", ESP_LOG_DEBUG);
            log("SD read test: %u-byte blocks, phase %u, continuous clock %u", options.blockBytes,
                options.delayPhase, unsigned(options.continuousClock));
            ReadHooks hooks; hooks.digest = &digest;
            ok = runSdReadTest(file, job.bytes, buffer, options.blockBytes, hooks, result);
            error = result.error;
            if (ok && mbedtls_sha256_finish(&digest, hash) != 0) { ok = false; error = "Cannot finish SD read digest"; }
        }
    }
    errno = 0;
    if (fclose(file) != 0) { ok = false; appendSdFileError(error, "Closing SD read test file failed", errno); }
    // Always restore before releasing exclusive SD access, including failures/cancellation.
    if (timingChanged) {
        const esp_err_t err = restoreSdReadTiming();
        if (err != ESP_OK) { ok = false; error += "; Cannot restore SD timing: " + std::string(esp_err_to_name(err)); }
    }
    esp_log_level_set("sdmmc_req", oldLogLevel);
    mbedtls_sha256_free(&digest);
    free(stream); free(buffer);
    if (ok) {
        char hex[65];
        for (size_t i = 0; i < sizeof(hash); ++i) snprintf(hex + i * 2, 3, "%02x", unsigned(hash[i]));
        Lock lock; job.sha256 = hex;
    }
    return ok;
}
void sdWorker(void*) {
    {
    SdState mounted;
    std::string error;
    SdBenchmarkResult result;
    SdScopeResult scopeResult;
    std::vector<SdScopeResult> scopeChannels;
    std::string restoreError;
    std::string wifiError;
    bool wifiOffDuringCapture = false, wifiRestored = false;
    int64_t wifiStoppedOffsetUs = 0, wifiResumedOffsetUs = 0;
    bool ok = false;
    if (job.scope) {
        mounted = job.beforeScope;
        // Allow the accepted HTTP response to leave before dropping both links.
        if (job.scopeOptions.wifiOff) vTaskDelay(pdMS_TO_TICKS(500));
        if (detachSdForScope(error)) {
            mounted = {};
            const bool radioReady = !job.scopeOptions.wifiOff || pauseWifiForScope(wifiError);
            if (job.scopeOptions.wifiOff && radioReady) {
                wifiStoppedOffsetUs = esp_timer_get_time() - job.started;
                vTaskDelay(pdMS_TO_TICKS(100)); // Allow the stop transient to settle.
            }
            if (radioReady && job.scopeOptions.pin == SdScopeAllPins) {
                ok = true;
                scopeChannels.reserve(6);
                for (const int pin : SdScopePins) {
                    if (cancel.load()) { scopeResult.cancelled = true; ok = false; break; }
                    { Lock lock; job.scopeActivePin = pin; }
                    scopeChannels.emplace_back();
                    auto& channel = scopeChannels.back();
                    channel.pin = pin;
                    SdScopeOptions selected = job.scopeOptions; selected.pin = pin;
                    if (!restoreSdScopePins(channel.error)) { ok = false; error = channel.error; break; }
                    const bool captured = captureSdScope(selected, cancel, channel, channel.error);
                    if (!captured) {
                        ok = false;
                        if (!channel.error.empty()) {
                            if (!error.empty()) error += "; ";
                            error += std::string(sdScopePinName(pin)) + ": " + channel.error;
                        }
                    }
                    { Lock lock; ++job.scopeCompleted; }
                    if (channel.cancelled) { scopeResult.cancelled = true; break; }
                }
            } else if (radioReady && restoreSdScopePins(error)) {
                { Lock lock; job.scopeActivePin = job.scopeOptions.pin; }
                ok = captureSdScope(job.scopeOptions, cancel, scopeResult, error);
                { Lock lock; job.scopeCompleted = 1; }
            }
            if (job.scopeOptions.wifiOff) {
                wifiOffDuringCapture = radioReady && wifiQuietForScope();
                if (radioReady && !wifiOffDuringCapture) wifiError = "Wi-Fi quiet period expired during capture";
                std::string resumeError;
                wifiRestored = resumeWifiAfterScope(resumeError);
                if (wifiRestored) wifiResumedOffsetUs = esp_timer_get_time() - job.started;
                if (!resumeError.empty()) {
                    if (!wifiError.empty()) wifiError += "; ";
                    wifiError += resumeError;
                }
                if (!wifiError.empty()) { ok = false; error = wifiError; }
            }
            // Always release the analog mux, even after configuration failure or cancellation.
            if (restoreSdScopePins(restoreError) && job.beforeScope.ready) {
                const auto& previous = job.beforeScope;
                remountSdCard(previous.width, previous.profileFrequency ? previous.profileFrequency : previous.frequency,
                              false, mounted, restoreError);
            }
            if (!restoreError.empty()) ok = false;
        }
    } else if (job.mount) {
        ok = remountSdCard(job.mode, job.maxFrequency, job.fallback, mounted, error);
    } else if (job.format) {
        ok = formatSdCard(job.mode, job.maxFrequency, job.fallback, mounted, error);
    } else if (job.readOnly) {
        ok = readTest(result, error);
    } else {
        const auto oldLogLevel = esp_log_level_get("sdmmc_req");
        const esp_err_t timing = configureSdReadTiming(job.timing.delayPhase, job.timing.continuousClock);
        esp_log_level_set("sdmmc_req", ESP_LOG_DEBUG);
        if (timing != ESP_OK) error = "Cannot set SD test timing: " + std::string(esp_err_to_name(timing));
        else {
            auto* buffer = static_cast<uint8_t*>(heap_caps_malloc(BlockBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            auto* streamBuffer = static_cast<char*>(heap_caps_malloc(BlockBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            FILE* file = nullptr;
            char path[80] = {};
            bool ownFile = false;
            if (!buffer || !streamBuffer) error = "Not enough internal memory for SD test";
            else {
                for (unsigned attempt = 0; attempt < 5; ++attempt) {
                    snprintf(path, sizeof(path), "/sdcard/.lpov-speed-%08lx.tmp", static_cast<unsigned long>(esp_random()));
                    // Newlib's fdopen requires F_GETFL, which IDF's FAT VFS does
                    // not implement. fopen's C11 'x' flag creates exclusively
                    // without that descriptor-to-stream conversion.
                    file = fopen(path, "w+bx");
                    if (file) { ownFile = true; break; }
                    if (errno != EEXIST) break;
                }
                if (!file) { const int code = errno; result.ioFailure = sdIoError(code); error = sdFileError("Cannot create SD test file", code); }
                else {
                    // Newlib's unbuffered fread can issue byte-sized VFS reads.
                    // A full block buffer measures SD transfers instead of that
                    // per-byte filesystem overhead. Flush time is still included.
                    if (setvbuf(file, streamBuffer, _IOFBF, BlockBytes) != 0) error = "Cannot configure SD block buffer";
                    else {
                        Hooks hooks;
                        ok = runSdBenchmark(file, job.bytes, buffer, BlockBytes, esp_random(), hooks, result);
                        error = result.error;
                    }
                    errno = 0;
                    if (fclose(file) != 0) {
                        const int code = errno;
                        result.ioFailure |= sdIoError(code); ok = false;
                        appendSdFileError(error, "Closing SD test file failed", code);
                    }
                }
            }
            free(buffer);
            free(streamBuffer);
            if (ownFile && unlink(path) != 0) {
                const int code = errno;
                result.ioFailure |= sdIoError(code);
                ok = false;
                appendSdFileError(error, "Cannot remove temporary test file", code);
                Lock lock; job.temporaryFile = path + 7; // Expose the SD-relative path.
            }
        }
        const esp_err_t restored = restoreSdReadTiming();
        if (restored != ESP_OK) { ok = false; error += "; Cannot restore SD timing: " + std::string(esp_err_to_name(restored)); }
        esp_log_level_set("sdmmc_req", oldLogLevel);
    }
    {
        Lock lock;
        if (job.format || job.mount || job.scope) { sd = mounted; job.width = sd.width; job.frequency = sd.frequency; }
        job.scopeResult = std::move(scopeResult); job.scopeChannels = std::move(scopeChannels);
        job.scopeActivePin = SdScopeAllPins; job.restoreError = restoreError;
        job.wifiOffDuringCapture = wifiOffDuringCapture; job.wifiRestored = wifiRestored;
        job.wifiStoppedOffsetUs = wifiStoppedOffsetUs; job.wifiResumedOffsetUs = wifiResumedOffsetUs;
        job.wifiError = wifiError;
        job.benchmark = result; job.error = error;
        if (job.readOnly && job.readOptions.keepWhiteBlink) {
            job.blinkTransitions = runtime.whiteBlinkTransitions - job.blinkTransitionsStart;
            job.blinkInterrupted = runtime.mode != Mode::WhiteBlink || !output.ready();
        }
        job.state = ok ? "complete" : (result.cancelled || job.scopeResult.cancelled) && error.empty() && restoreError.empty() ? "cancelled" : "failed";
        job.elapsed = esp_timer_get_time() - job.started;
        runtime.maintenance = false; runtime.lastActivity = esp_timer_get_time();
        // Keep automatic/background playback from restarting while the trace is inspected.
        // Starting playback or another diagnostic explicitly clears this existing idle latch.
        if (job.scope) runtime.playbackComplete = true;
        log("SD %s: %s%s%s", job.scope ? "scope" : job.mount ? "mount" : job.format ? "format" : job.readOnly ? "read test" : "speed test", job.state,
            error.empty() ? "" : ": ", error.c_str());
        if (!restoreError.empty()) { runtime.error = restoreError; log("SD scope: %s", restoreError.c_str()); }
        running = false;
        // Settings were committed to NVS before mounting. Keep SD backup
        // writes out of this completion lock; a bad card must not block it.
        if (job.mount && ok) runtime.error.clear();
        if (!job.format && !job.mount && !job.readOnly && !job.scope && result.ioFailure) requestSdRecovery(error);
    }
    } // Destroy result/error allocations before deleting this FreeRTOS task.
    vTaskDelete(nullptr);
}
}

bool sdToolRunning() { return running.load(); }
void cancelSdSpeedTest() { if (running.load() && !recovery.active && !job.format && !job.mount) cancel = true; }
bool requestSdRecovery(const std::string& reason, bool manual) {
    if (!config.sdFallback || (!manual && !config.sdRecoverErrors) || !sd.ready || running.load() || runtime.maintenance) return false;
    const auto profiles = sdProfiles(config.sdMode, config.sdFrequency, true);
    if (manual && profiles.after({sd.width, sd.profileFrequency}) >= profiles.count) return false;
    recovery.failed = sd; recovery.mode = config.sdMode; recovery.maximum = config.sdFrequency;
    recovery.reason = reason; recovery.error.clear(); recovery.attempts = 0;
    recovery.width = 0; recovery.frequency = 0;
    recovery.started = esp_timer_get_time(); recovery.elapsed = 0;
    recovery.active = true; recovery.pending = true; recovery.state = "waiting for SD read";
    stopPlayback(); sequence.close();
    sd.ready = false; runtime.maintenance = true; running = true;
    runtime.error = reason + "; SD recovery queued.";
    log("%s", runtime.error.c_str());
    return true;
}
void serviceSdRecovery() {
    if (!recovery.pending || !sequence.quiescent()) return;
    recovery.pending = false; recovery.state = "recovering";
    if (xTaskCreatePinnedToCore(recoveryWorker, "sd_recovery", 8192, nullptr, 1, nullptr, 0) != pdPASS) {
        recovery.active = false; recovery.state = "failed";
        recovery.error = runtime.error = "Cannot start SD recovery worker; retry mounting";
        runtime.maintenance = false; running = false;
    }
}
void sdRecoveryAttempt(unsigned width, unsigned frequency) {
    ++recovery.attempts; recovery.width = width; recovery.frequency = frequency;
    log("SD recovery: trying %u-bit/%u kHz", width, frequency);
}
cJSON* sdRecoveryJson() {
    auto* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "running", recovery.active);
    cJSON_AddStringToObject(j, "state", recovery.state);
    cJSON_AddStringToObject(j, "reason", recovery.reason.c_str());
    cJSON_AddStringToObject(j, "error", recovery.error.c_str());
    cJSON_AddNumberToObject(j, "attempts", recovery.attempts);
    cJSON_AddNumberToObject(j, "successes", recovery.successes);
    cJSON_AddNumberToObject(j, "width", recovery.width);
    cJSON_AddNumberToObject(j, "frequency", recovery.frequency);
    cJSON_AddNumberToObject(j, "elapsed_ms", (recovery.active ? esp_timer_get_time() - recovery.started : recovery.elapsed) / 1000);
    return j;
}
static bool startSdJob(bool format, bool mount, unsigned mebibytes, std::string& error,
                       const SdReadTestOptions* readOptions = nullptr, const SdTimingOptions& timing = {},
                       const SdScopeOptions* scopeOptions = nullptr) {
    if (running.load() || runtime.maintenance) { error = "Storage maintenance is already running"; return false; }
    if (!format && !mount && !scopeOptions && !sd.ready) { error = "Mount the SD card before running its speed test"; return false; }
    if (!format && !mount && !scopeOptions && mebibytes != 1 && mebibytes != 4 && mebibytes != 16) {
        error = "Choose a 1, 4, or 16 MiB test"; return false;
    }
    const bool keepBlink = readOptions && readOptions->keepWhiteBlink;
    if (keepBlink && (runtime.mode != Mode::WhiteBlink || !output.ready() || !config.brightness)) {
        error = "Start white blinking above 0% brightness before keeping it during an SD read"; return false;
    }
    if (!keepBlink) stopPlayback();
    sequence.close();
    if (!sequence.quiescent()) { error = "SD read is still finishing; retry shortly"; return false; }
    job = {};
    job.format = format; job.mount = mount; job.state = "running";
    job.timing = timing;
    if (readOptions) {
        job.readOnly = true; job.readOptions = *readOptions;
        job.blinkTransitionsStart = runtime.whiteBlinkTransitions;
    }
    if (scopeOptions) { job.scope = true; job.scopeOptions = *scopeOptions; job.beforeScope = sd; }
    job.bytes = format || mount ? 0 : mebibytes * 1024U * 1024U;
    job.width = sd.width; job.frequency = sd.frequency;
    job.mode = config.sdMode; job.maxFrequency = config.sdFrequency;
    job.fallback = config.sdFallback;
    job.started = esp_timer_get_time();
    const SdState previous = sd;
    if (format || mount || scopeOptions) sd = {};
    runtime.maintenance = true; running = true; cancel = false;
    if (xTaskCreatePinnedToCore(sdWorker, "sd_tools", 8192, nullptr, 1, nullptr, 0) != pdPASS) {
        running = false; runtime.maintenance = false; sd = previous;
        job.state = "failed"; job.error = error = "Cannot start SD worker"; return false;
    }
    return true;
}
bool startSdTool(bool format, unsigned mebibytes, std::string& error, const SdTimingOptions& timing) {
    if (timing.delayPhase > 3) { error = "Invalid SD timing phase"; return false; }
    return startSdJob(format, false, mebibytes, error, nullptr, timing);
}
bool startSdMount(std::string& error) { return startSdJob(false, true, 0, error); }
bool startSdScope(const SdScopeOptions& options, std::string& error) {
    if ((options.pin != SdScopeAllPins && !sdScopePinName(options.pin)) ||
        (options.intervalUs != 100 && options.intervalUs != 1000 && options.intervalUs != 10000)) {
        error = "Select an SD pin and a 100, 1000, or 10000 us sample interval"; return false;
    }
    return startSdJob(false, false, 0, error, nullptr, {}, &options);
}
bool startSdReadTest(unsigned mebibytes, const SdReadTestOptions& options, std::string& error) {
    std::string path;
    if (!sdPath(options.path, path) || options.path == "/" || options.delayPhase > 3 ||
        (options.blockBytes != 512 && options.blockBytes != 4096 && options.blockBytes != 16384)) {
        error = "Invalid SD read test options"; return false;
    }
    return startSdJob(false, false, mebibytes, error, &options, options);
}
cJSON* sdToolJson() {
    cJSON* j = cJSON_CreateObject();
    const auto& b = job.benchmark;
    cJSON_AddBoolToObject(j, "running", running.load());
    cJSON_AddNumberToObject(j, "captureId", job.scope ? job.started : 0);
    if (job.scope) {
        cJSON_AddNumberToObject(j, "scopeChannelsTarget", job.scopeOptions.pin == SdScopeAllPins ? 6 : 1);
        cJSON_AddNumberToObject(j, "scopeChannelsCompleted", job.scopeCompleted);
        cJSON_AddBoolToObject(j, "scopeWifiOff", job.scopeOptions.wifiOff);
        cJSON_AddStringToObject(j, "scopeActiveSignal", sdScopePinName(job.scopeActivePin) ? sdScopePinName(job.scopeActivePin) : "");
    }
    cJSON_AddStringToObject(j, "operation", recovery.active ? "recovery" : job.scope ? "scope" : job.mount ? "mount" : job.format ? "format" : job.readOnly ? "read" : "speed");
    cJSON_AddStringToObject(j, "state", recovery.active ? "running" : job.state);
    cJSON_AddStringToObject(j, "phase", recovery.active ? recovery.state : job.scope ? "capture / restore" : job.mount ? "mounting" : job.format ? "formatting" : b.phase);
    cJSON_AddStringToObject(j, "error", job.error.c_str());
    cJSON_AddStringToObject(j, "temporaryFile", job.temporaryFile.c_str());
    cJSON_AddNumberToObject(j, "bytesTarget", job.bytes);
    cJSON_AddNumberToObject(j, "bytesWritten", b.written);
    cJSON_AddNumberToObject(j, "bytesRead", b.read);
    cJSON_AddNumberToObject(j, "blockBytes", job.readOnly ? job.readOptions.blockBytes : BlockBytes);
    cJSON_AddNumberToObject(j, "delayPhase", job.timing.delayPhase);
    cJSON_AddBoolToObject(j, "continuousClock", job.timing.continuousClock);
    if (job.readOnly) {
        const bool blink = job.readOptions.keepWhiteBlink;
        cJSON_AddBoolToObject(j, "whiteBlinkEnabled", blink);
        cJSON_AddNumberToObject(j, "whiteBlinkTransitions", blink && running.load()
            ? runtime.whiteBlinkTransitions - job.blinkTransitionsStart : job.blinkTransitions);
        cJSON_AddBoolToObject(j, "whiteBlinkInterrupted", blink && running.load()
            ? runtime.mode != Mode::WhiteBlink || !output.ready() : job.blinkInterrupted);
        cJSON_AddStringToObject(j, "path", job.readOptions.path.c_str());
        cJSON_AddStringToObject(j, "sha256", job.sha256.c_str());
    }
    cJSON_AddNumberToObject(j, "busWidth", job.width);
    cJSON_AddNumberToObject(j, "clockKHz", job.frequency);
    cJSON_AddNumberToObject(j, "writeMiBps", b.writeUs ? double(b.written) * 1000000 / (1048576 * double(b.writeUs)) : 0);
    cJSON_AddNumberToObject(j, "readMiBps", b.readUs ? double(b.read) * 1000000 / (1048576 * double(b.readUs)) : 0);
    cJSON_AddNumberToObject(j, "maxWrite_ms", b.maxWriteUs / 1000.0);
    cJSON_AddNumberToObject(j, "maxRead_ms", b.maxReadUs / 1000.0);
    cJSON_AddBoolToObject(j, "verified", b.verified);
    const auto elapsed = recovery.active ? esp_timer_get_time() - recovery.started :
        running.load() ? esp_timer_get_time() - job.started : job.elapsed;
    cJSON_AddNumberToObject(j, "elapsed_ms", elapsed / 1000);
    return j;
}

static void addScopeTrace(cJSON* j, const SdScopeResult& capture) {
    cJSON_AddNumberToObject(j, "adcUnit", capture.adcUnit);
    cJSON_AddBoolToObject(j, "calibrated", capture.calibrated);
    cJSON_AddNumberToObject(j, "startOffset_us", capture.startedUs ? capture.startedUs - job.started : 0);
    cJSON_AddStringToObject(j, "calibrationError", capture.calibrationError.c_str());
    cJSON_AddStringToObject(j, "readError", capture.readError.c_str());
    // Six 1,024-point traces would otherwise create >30,000 small cJSON nodes.
    // A bounded raw array keeps the bulk allocation in PSRAM, preserving Wi-Fi RAM.
    std::string points = "[";
    points.reserve(capture.points.size() * 40 + 2);
    for (const auto& point : capture.points) {
        if (points.size() > 1) points += ',';
        points += '[' + std::to_string(point.timeUs) + ',';
        points += point.raw >= 0 ? std::to_string(point.raw) : "null";
        points += ',';
        points += point.millivolts >= 0 ? std::to_string(point.millivolts) : "null";
        points += point.upperLimit ? ",true]" : ",false]";
    }
    points += ']';
    cJSON_AddItemToObject(j, "samples", cJSON_CreateRaw(points.c_str()));
}

cJSON* sdScopeJson() {
    auto* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "running", job.scope && !recovery.active && running.load());
    cJSON_AddBoolToObject(j, "storageBusy", running.load());
    cJSON_AddStringToObject(j, "state", job.scope ? job.state : "idle");
    cJSON_AddNumberToObject(j, "captureId", job.scope ? job.started : 0);
    if (!job.scope) return j;
    const bool all = job.scopeOptions.pin == SdScopeAllPins;
    cJSON_AddStringToObject(j, "signal", all ? "All SD pins" : sdScopePinName(job.scopeOptions.pin));
    cJSON_AddStringToObject(j, "captureMode", all ? "sequential" : "single");
    cJSON_AddNumberToObject(j, "channelsTarget", all ? 6 : 1);
    cJSON_AddNumberToObject(j, "pin", job.scopeOptions.pin);
    cJSON_AddNumberToObject(j, "requestedInterval_us", job.scopeOptions.intervalUs);
    cJSON_AddNumberToObject(j, "samplesTarget", SdScopeSamples);
    cJSON_AddBoolToObject(j, "wifiOffRequested", job.scopeOptions.wifiOff);
    cJSON_AddBoolToObject(j, "wifiOffDuringCapture", job.wifiOffDuringCapture);
    cJSON_AddBoolToObject(j, "wifiRestored", job.wifiRestored);
    cJSON_AddNumberToObject(j, "wifiStoppedOffset_us", job.wifiStoppedOffsetUs);
    cJSON_AddNumberToObject(j, "wifiResumedOffset_us", job.wifiResumedOffsetUs);
    cJSON_AddStringToObject(j, "wifiError", job.wifiError.c_str());
    cJSON_AddBoolToObject(j, "wasMounted", job.beforeScope.ready);
    cJSON_AddBoolToObject(j, "sdReady", sd.ready);
    cJSON_AddStringToObject(j, "error", job.error.c_str());
    cJSON_AddStringToObject(j, "restoreError", job.restoreError.c_str());
    if (all) {
        auto* channels = cJSON_AddArrayToObject(j, "channels");
        for (const auto& channel : job.scopeChannels) {
            auto* item = cJSON_CreateObject();
            cJSON_AddStringToObject(item, "signal", sdScopePinName(channel.pin));
            cJSON_AddNumberToObject(item, "pin", channel.pin);
            cJSON_AddStringToObject(item, "error", channel.error.c_str());
            cJSON_AddBoolToObject(item, "cancelled", channel.cancelled);
            addScopeTrace(item, channel);
            cJSON_AddItemToArray(channels, item);
        }
    } else {
        addScopeTrace(j, job.scopeResult);
    }
    return j;
}
}
