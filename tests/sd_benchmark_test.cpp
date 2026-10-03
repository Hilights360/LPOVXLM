#include "sd_benchmark.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <iostream>

struct Hooks {
    FILE* file;
    bool stop = false, corrupt = false, failFlush = false, timeout = false;
    int flushCode = EIO;
    uint32_t cancelAfter = 0;
    int64_t time = 0;
    int64_t now() { errno = EINVAL; time += timeout ? 61000000 : 100; return time; }
    bool cancelled() { return stop; }
    bool sync(FILE* f) { if (failFlush) { errno = flushCode; return false; } return fflush(f) == 0; }
    void progress(const pov::SdBenchmarkResult& r) {
        if (cancelAfter && r.written >= cancelAfter) stop = true;
        if (corrupt && std::string(r.phase) == "reading") {
            // Corruption on disk must not be mistaken for a successful read.
            assert(fseek(file, 17, SEEK_SET) == 0);
            int original = fgetc(file);
            assert(fseek(file, 17, SEEK_SET) == 0);
            assert(fputc(original ^ 0xff, file) != EOF);
            assert(fflush(file) == 0);
            assert(fseek(file, 0, SEEK_SET) == 0);
            corrupt = false;
        }
    }
};
int main() {
    constexpr uint32_t bytes = 65537; // Exercise a partial final block.
    std::array<uint8_t, 16384> buffer{};
    for (int scenario = 0; scenario < 6; ++scenario) {
        FILE* f = tmpfile(); assert(f);
        Hooks hooks{f};
        hooks.cancelAfter = scenario == 1 ? 16384 : 0;
        hooks.corrupt = scenario == 2;
        hooks.failFlush = scenario == 3 || scenario == 5;
        hooks.flushCode = scenario == 5 ? ENOSPC : EIO;
        hooks.timeout = scenario == 4;
        pov::SdBenchmarkResult result;
        const bool ok = pov::runSdBenchmark(f, bytes, buffer.data(), buffer.size(), 123456, hooks, result);
        if (scenario == 0) {
            assert(ok && result.verified && result.error.empty());
            assert(result.written == bytes && result.read == bytes);
            assert(result.writeUs > 0 && result.readUs > 0 && result.maxReadUs > 0);
            assert(pov::sdTestByte(0, 123456) != pov::sdTestByte(16384, 123456));
        } else {
            assert(!ok && !result.verified);
            if (scenario == 1) assert(result.cancelled && result.written == 16384 && result.read == 0);
            if (scenario == 2) assert(result.error == "SD verification failed at byte 17");
            if (scenario == 3 || scenario == 5) {
                // Clock/progress helpers can change errno; preserve the value
                // from the failing I/O and retain it through cleanup failures.
                const std::string primary = pov::sdFileError("SD flush failed", hooks.flushCode);
                assert(result.error == primary);
                std::string error = result.error;
                pov::appendSdFileError(error, "Closing SD test file failed", EIO);
                pov::appendSdFileError(error, "Cannot remove temporary test file", EIO);
                assert(error.find(primary + "; Closing SD test file failed: ") == 0);
                assert(error.find("; Cannot remove temporary test file: ") != std::string::npos);
            }
            if (scenario == 4) assert(result.error == "SD test exceeded 60 seconds");
            assert(result.ioFailure == (scenario == 2 || scenario == 3));
        }
        fclose(f);
    }
    std::cout << "SD benchmark passed: verification, partial block, cancellation, corruption, preserved I/O errors, full card classification and deadline\n";
}
