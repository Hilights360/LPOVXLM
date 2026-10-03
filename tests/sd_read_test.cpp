#include "sd_read_test.hpp"
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

struct ReadHooks {
    bool stop = false, timeout = false, cancelAfterProgress = false;
    int64_t time = 0;
    std::vector<uint8_t> received;
    int64_t now() { errno = EINVAL; time += timeout ? 61000000 : 100; return time; }
    bool cancelled() { return stop; }
    bool consume(const uint8_t* data, size_t size) { received.insert(received.end(), data, data + size); return true; }
    void progress(const pov::SdBenchmarkResult&) { if (cancelAfterProgress) stop = true; }
};

int main() {
    std::vector<uint8_t> original(65537);
    for (size_t i = 0; i < original.size(); ++i) original[i] = uint8_t(i * 37 + i / 127);
    std::array<uint8_t, 16384> buffer{};
    for (unsigned block : {512U, 4096U, 16384U}) {
        for (unsigned scenario = 0; scenario < 4; ++scenario) {
            FILE* file = tmpfile(); assert(file);
            assert(fwrite(original.data(), 1, original.size(), file) == original.size());
            rewind(file);
            ReadHooks hooks;
            hooks.cancelAfterProgress = scenario == 1;
            hooks.timeout = scenario == 3;
            pov::SdBenchmarkResult result;
            const bool ok = pov::runSdReadTest(file, unsigned(original.size()) + (scenario == 2 ? 1 : 0),
                                               buffer.data(), block, hooks, result);
            assert(result.written == 0 && !result.verified);
            if (scenario == 0) {
                assert(ok && result.phase == std::string("done"));
                assert(hooks.received == original && result.read == original.size());
            } else if (scenario == 1) {
                assert(!ok && result.cancelled && result.read == 16384);
            } else if (scenario == 2) {
                assert(!ok && result.read == original.size() && !result.ioFailure);
                assert(result.error.find("incomplete") != std::string::npos);
            } else {
                assert(!ok && result.read == 0 && result.error.find("60 seconds") != std::string::npos);
            }
            rewind(file);
            std::vector<uint8_t> after(original.size());
            assert(fread(after.data(), 1, after.size(), file) == after.size());
            assert(after == original); // Read tests never alter their input, including failed/cancelled runs.
            fclose(file);
        }
    }
    std::cout << "SD read tests passed: complete/partial reads, fixed progress cadence, cancellation, deadline and unchanged source\n";
}
