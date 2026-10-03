#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include "sd_recovery.hpp"

namespace pov {
inline std::string sdFileError(const std::string& operation, int code) {
    return operation + (code ? ": " + std::string(strerror(code)) +
        " (errno " + std::to_string(code) + ")" : ": no filesystem error code");
}
inline void appendSdFileError(std::string& error, const char* operation, int code) {
    if (!error.empty()) error += "; ";
    error += sdFileError(operation, code);
}
struct SdBenchmarkResult {
    uint32_t written = 0, read = 0;
    uint64_t writeUs = 0, readUs = 0, maxWriteUs = 0, maxReadUs = 0;
    const char* phase = "writing";
    bool verified = false, cancelled = false, ioFailure = false;
    std::string error;
};
inline uint8_t sdTestByte(uint32_t position, uint32_t seed) {
    uint32_t mixed = seed ^ (position * 0x9e3779b9U);
    mixed ^= mixed >> 16;
    return static_cast<uint8_t>(mixed);
}

// Hooks provide the clock, cancellation, durable flush and progress/yield.
// The caller owns an exclusively created temporary file and cleans it up.
template<class Hooks>
bool runSdBenchmark(FILE* file, uint32_t bytes, uint8_t* buffer, size_t block,
                    uint32_t seed, Hooks& hooks, SdBenchmarkResult& result) {
    const int64_t started = hooks.now();
    auto interrupted = [&]() {
        if (hooks.cancelled()) { result.cancelled = true; return true; }
        if (hooks.now() - started > 60000000) {
            result.error = "SD test exceeded 60 seconds"; return true;
        }
        return false;
    };
    if (!file || !buffer || !bytes || !block) { result.error = "Invalid test buffers"; return false; }
    hooks.progress(result);
    while (result.written < bytes) {
        if (interrupted()) return false;
        const size_t count = std::min<size_t>(block, bytes - result.written);
        for (size_t i = 0; i < count; ++i) buffer[i] = sdTestByte(result.written + static_cast<uint32_t>(i), seed);
        const int64_t begin = hooks.now();
        errno = 0;
        const size_t done = fwrite(buffer, 1, count, file);
        const int code = errno;
        const uint64_t elapsed = hooks.now() - begin;
        result.writeUs += elapsed; result.maxWriteUs = std::max(result.maxWriteUs, elapsed);
        result.written += static_cast<uint32_t>(done);
        if (done != count) {
            result.ioFailure = sdIoError(code);
            result.error = sdFileError("SD write failed at byte " + std::to_string(result.written), code);
            return false;
        }
        hooks.progress(result);
    }
    if (interrupted()) return false;
    result.phase = "flushing"; hooks.progress(result);
    const int64_t flushStart = hooks.now();
    errno = 0;
    const bool flushed = hooks.sync(file);
    const int flushCode = errno;
    const uint64_t flushUs = hooks.now() - flushStart;
    result.writeUs += flushUs; result.maxWriteUs = std::max(result.maxWriteUs, flushUs);
    if (!flushed) { result.ioFailure = sdIoError(flushCode); result.error = sdFileError("SD flush failed", flushCode); return false; }
    errno = 0;
    if (fseek(file, 0, SEEK_SET) != 0) {
        const int code = errno;
        result.ioFailure = sdIoError(code); result.error = sdFileError("Cannot rewind SD test file", code); return false;
    }
    result.phase = "reading"; hooks.progress(result);
    while (result.read < bytes) {
        if (interrupted()) return false;
        const size_t count = std::min<size_t>(block, bytes - result.read);
        const int64_t begin = hooks.now();
        errno = 0;
        const size_t done = fread(buffer, 1, count, file);
        const int code = errno;
        const uint64_t elapsed = hooks.now() - begin;
        result.readUs += elapsed; result.maxReadUs = std::max(result.maxReadUs, elapsed);
        if (done != count) {
            result.ioFailure = true;
            result.error = sdFileError("SD read incomplete at byte " + std::to_string(result.read + done), code);
            return false;
        }
        for (size_t i = 0; i < count; ++i) {
            if (buffer[i] != sdTestByte(result.read + static_cast<uint32_t>(i), seed)) {
                result.ioFailure = true;
                result.error = "SD verification failed at byte " + std::to_string(result.read + i);
                return false;
            }
        }
        result.read += static_cast<uint32_t>(done);
        hooks.progress(result);
    }
    if (interrupted()) return false;
    result.verified = true; result.phase = "done";
    return true;
}
}
