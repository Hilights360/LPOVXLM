#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

namespace pov {
constexpr char DefaultHostname[] = "lpov";

// Upgrade only this device's generated defaults; preserve custom names.
// The native port used the last three MAC bytes. Arduino used the low
// 24 bits of its little-endian efuse MAC value (bytes 2, 1, 0).
inline std::string hostnameAfterUpgrade(const std::string& saved, const uint8_t mac[6]) {
    if (saved.empty()) return DefaultHostname;
    std::string lower = saved;
    for (char& c : lower) if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    char native[16], arduino[16];
    std::snprintf(native, sizeof(native), "pov-%02x%02x%02x",
        unsigned(mac[3]), unsigned(mac[4]), unsigned(mac[5]));
    std::snprintf(arduino, sizeof(arduino), "pov-%02x%02x%02x",
        unsigned(mac[2]), unsigned(mac[1]), unsigned(mac[0]));
    return lower == native || lower == arduino ? DefaultHostname : saved;
}
}
