#pragma once
#include "sd_benchmark.hpp"

namespace pov {
struct SdTimingOptions {
    unsigned delayPhase = 0;
    bool continuousClock = false;
};
struct SdReadTestOptions : SdTimingOptions {
    std::string path;
    unsigned blockBytes = 16384;
    bool keepWhiteBlink = false; // Explicitly retain the SD-independent LED load.
};

// Read existing bytes only. A completed read produces a digest through Hooks;
// it is not marked verified until a caller compares it with a known reference.
template<class Hooks>
bool runSdReadTest(FILE* file, uint32_t bytes, uint8_t* buffer, size_t block,
                   Hooks& hooks, SdBenchmarkResult& result) {
    result.phase = "reading";
    if (!file || !buffer || !bytes || !block) { result.error = "Invalid read test buffers"; return false; }
    const int64_t started = hooks.now();
    uint32_t lastProgress = 0;
    while (result.read < bytes) {
        if (hooks.cancelled()) { result.cancelled = true; return false; }
        if (hooks.now() - started > 60000000) { result.error = "SD read test exceeded 60 seconds"; return false; }
        const size_t count = std::min<size_t>(block, bytes - result.read);
        const int64_t begin = hooks.now();
        errno = 0;
        const size_t done = fread(buffer, 1, count, file);
        const int code = errno;
        const uint64_t elapsed = hooks.now() - begin;
        result.readUs += elapsed;
        result.maxReadUs = std::max(result.maxReadUs, elapsed);
        result.read += static_cast<uint32_t>(done);
        if (done && !hooks.consume(buffer, done)) { result.error = "SD read digest failed"; return false; }
        if (done != count) {
            result.ioFailure = ferror(file) != 0;
            result.error = sdFileError("SD read incomplete at byte " + std::to_string(result.read), code);
            return false;
        }
        // Keep scheduling pauses independent of the chosen read block size.
        if (result.read - lastProgress >= 16384 || result.read == bytes) {
            hooks.progress(result);
            lastProgress = result.read;
        }
    }
    result.phase = "done";
    return true;
}
}
