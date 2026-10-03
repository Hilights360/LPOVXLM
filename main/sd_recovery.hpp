#pragma once
#include <array>
#include <cstddef>
#include <cerrno>

namespace pov {
constexpr unsigned SdDefaultFrequency = 20000;
// IDF programs the SDMMC data-read timeout to 100 ms of card clock. This card
// occasionally needs longer for a single sector, and the resulting
// ESP_ERR_TIMEOUT on CMD17 leaves it out of step, so the immediate retry fails
// too. Allowing a slow sector to simply arrive late is what keeps playback up.
constexpr unsigned SdDataTimeoutMs = 300;
constexpr std::array<unsigned, 3> SdFrequencies{40000, 20000, 400};
inline unsigned normalizedSdFrequency(unsigned khz) {
    for (unsigned allowed : SdFrequencies) if (khz == allowed) return khz;
    // Retire the old 1-10 MHz settings: IDF 5.3.3 uses host divider 2 there,
    // reducing the nominal output phase delay to 3.125 ns on this S3 board.
    return SdDefaultFrequency;
}
struct SdProfile { unsigned width = 0, khz = 0; };
struct SdProfiles {
    std::array<SdProfile, 4> values{};
    size_t count = 0;
    // A recovery pass always starts AFTER the profile that failed. Never wrap
    // back to faster settings or loop forever at the slowest setting.
    size_t after(SdProfile failed) const {
        for (size_t i = 0; i < count; ++i)
            if (values[i].width == failed.width && values[i].khz == failed.khz) return i + 1;
        return count;
    }
};
inline SdProfiles sdProfiles(unsigned mode, unsigned maximumKHz, bool fallback) {
    SdProfiles result;
    const unsigned firstWidth = mode == 1 ? 1 : 4;
    for (unsigned width : {4U, 1U}) {
        if (width > firstWidth || (!fallback && width != firstWidth)) continue;
        for (unsigned khz : SdFrequencies) {
            if (khz > maximumKHz) continue;
            // Prefer 1-bit/20 MHz over crawling along at 4-bit/400 kHz.
            // Once in backup mode, do not go back up to 40 MHz or 4-bit.
            if (width == 4 && khz == 400 && maximumKHz >= SdDefaultFrequency) continue;
            if (firstWidth == 4 && width == 1 && khz > SdDefaultFrequency) continue;
            result.values[result.count++] = {width, khz};
            if (!fallback) return result;
        }
    }
    return result;
}
inline bool sdIoError(int code) {
    // Full cards, missing paths and malformed files are not bus-speed faults.
    return code == EIO || code == ETIMEDOUT || code == ENODEV;
}
}
