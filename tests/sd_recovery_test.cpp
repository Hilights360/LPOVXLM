#include "sd_recovery.hpp"
#include <cassert>
#include <iostream>

int main() {
    using namespace pov;
    assert(SdDefaultFrequency == 20000);
    for (unsigned old : {10000U, 8000U, 4000U, 1000U, 0U, 12345U})
        assert(normalizedSdFrequency(old) == 20000);
    for (unsigned allowed : SdFrequencies) assert(normalizedSdFrequency(allowed) == allowed);
    const auto profiles = sdProfiles(4, SdDefaultFrequency, true);
    assert(profiles.count == 3);
    const SdProfile expected[] = {{4, 20000}, {1, 20000}, {1, 400}};
    for (size_t i = 0; i < profiles.count; ++i) {
        assert(profiles.values[i].width == expected[i].width);
        assert(profiles.values[i].khz == expected[i].khz);
        assert(profiles.after(profiles.values[i]) == i + 1);
    }
    // A failed 4-bit transfer must go straight to the full-speed 1-bit backup.
    auto resume = profiles.after({4, 20000});
    assert(resume == 1 && profiles.values[resume].width == 1 && profiles.values[resume].khz == 20000);
    resume = profiles.after(profiles.values[resume]);
    assert(resume == 2 && profiles.values[resume].width == 1 && profiles.values[resume].khz == 400);
    assert(profiles.after(profiles.values[resume]) == profiles.count); // Exhaustion cannot wrap.
    assert(profiles.after({4, 1234}) == profiles.count); // Unknown profile also cannot restart faster.
    for (unsigned mode : {0U, 1U, 4U}) {
        const auto strict = sdProfiles(mode, 20000, false);
        assert(strict.count == 1 && strict.values[0].khz == 20000);
        assert(strict.values[0].width == (mode == 1 ? 1U : 4U));
    }
    const auto oneBit = sdProfiles(1, 20000, true);
    assert(oneBit.count == 2 && oneBit.values[0].khz == 20000 && oneBit.values[1].khz == 400);
    for (size_t i = 0; i < oneBit.count; ++i) assert(oneBit.values[i].width == 1);
    const auto fast = sdProfiles(4, 40000, true);
    assert(fast.count == 4 && fast.values[0].width == 4 && fast.values[0].khz == 40000);
    assert(fast.values[1].width == 4 && fast.values[1].khz == 20000);
    assert(fast.values[2].width == 1 && fast.values[2].khz == 20000);
    assert(fast.values[3].width == 1 && fast.values[3].khz == 400);
    for (unsigned mode : {0U, 1U, 4U}) {
        for (unsigned maximum : {400U, 1000U, 4000U, 8000U, 10000U, 20000U, 40000U}) {
            for (bool fallback : {false, true}) {
                const auto choices = sdProfiles(mode, maximum, fallback);
                assert(choices.count && choices.count <= choices.values.size());
                if (!fallback) assert(choices.count == 1);
                for (size_t i = 0; i < choices.count; ++i) {
                    const auto current = choices.values[i];
                    assert(current.khz <= maximum && normalizedSdFrequency(current.khz) == current.khz);
                    if (mode == 1) assert(current.width == 1);
                    if (i) {
                        assert(current.width <= choices.values[i - 1].width);
                        assert(current.khz <= choices.values[i - 1].khz);
                    }
                }
                assert(choices.after(choices.values[choices.count - 1]) == choices.count);
            }
        }
    }
    assert(sdIoError(EIO) && sdIoError(ETIMEDOUT) && sdIoError(ENODEV));
    assert(!sdIoError(ENOSPC) && !sdIoError(ENOENT) && !sdIoError(EINVAL) && !sdIoError(0));
    std::cout << "SD recovery passed: ordered fallback, 1-bit transition, resume/exhaustion, strict mode and I/O error classification\n";
}
