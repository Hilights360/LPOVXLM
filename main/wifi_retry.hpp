#pragma once
#include <cstdint>

namespace pov {
// Connection attempts share the AP radio. An explicit Save/Retry permits one
// attempt while a local client is connected; automatic retries wait.
struct WifiRetry {
    static constexpr unsigned Limit = 3;
    unsigned attempts = 0;
    int64_t nextAt = 0;
    bool enabled = false, explicitAttempt = false;
    void reset(bool enable, bool manual, int64_t now) {
        enabled = enable; explicitAttempt = manual; attempts = 0;
        nextAt = now + 500000; // Let STA start/stop events drain first.
    }
    bool ready(int64_t now, unsigned apClients) const {
        return enabled && attempts < Limit && now >= nextAt && (!apClients || explicitAttempt);
    }
    void started() { ++attempts; explicitAttempt = false; }
    void failed(int64_t now) { nextAt = now + (attempts <= 1 ? 5000000 : 15000000); }
    void connected() { attempts = 0; explicitAttempt = false; }
};
}
