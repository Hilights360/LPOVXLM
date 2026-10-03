#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "BoardPins.h"
#include "SharedClockProtocol.h"
#include "fseq_format.hpp"
#include "fseq.hpp"
#include "hostname.hpp"
#include "sd_read_test.hpp"
#include "sd_recovery.hpp"
#include "sd_scope.hpp"
#include "arm_wiring.hpp"
#include "playback_timing.hpp"
#include "fseq_settings.hpp"
#include "hall_filter.hpp"
#include "radial_brightness.hpp"

namespace pov {
constexpr unsigned MaxArms = 4;
constexpr unsigned MaxPixels = 1024;
constexpr char AccessPointName[] = "POV-Spinner";
constexpr char AccessPointPassword[] = "POV123456";
struct Settings {
    uint8_t brightness = 25, duty = 60, arms = 4, sdMode = 4;
    uint8_t centerBrightness = 10;
    bool centerDimming = false;
    bool autoDuty = false;
    bool autoFseq = false;
    uint8_t ledProtocol = 0; // SharedClockProtocol::Protocol; retain SK9822 framing by default.
    uint16_t fps = 40, spokes = 40, pixels = 144;
    uint32_t startChannel = 1, sdFrequency = SdDefaultFrequency;
    bool sdFallback = true, sdRecoverErrors = true;
    bool autoplay = true, watchdog = false, background = false;
    bool loop = true;
    bool perArm = false, strobe = false;
    bool armClockwise = false, rotationClockwise = true; // Preserve the original CCW arm geometry.
    float strobeWidth = 3, phase = 0;
    std::array<uint32_t, MaxArms> starts{1, 433, 865, 1297};
    std::array<float, MaxArms> armPhase{};
    std::string ssid, password, hostname = DefaultHostname, backgroundPath;
};
extern Settings config;
extern SemaphoreHandle_t stateMutex;
// Settings and published frames are serialized. Frame I/O/decompression runs
// outside this lock so the display can meet its blanking deadlines.
struct Lock {
    Lock() { xSemaphoreTakeRecursive(stateMutex, portMAX_DELAY); }
    ~Lock() { xSemaphoreGiveRecursive(stateMutex); }
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
};
void log(const char* format, ...) __attribute__((format(printf, 1, 2)));
std::string logText();
void clearLogs();
esp_err_t loadSettings();
esp_err_t saveSettings();
void normalizeSettings();
void restoreSettingsBackup();
bool saveSettingsBackup();
bool loadSequenceTipFirst(const std::string& path);
esp_err_t saveSequenceTipFirst(const std::string& path, bool tipFirst);

struct SdState {
    bool ready = false;
    unsigned width = 0, frequency = 0;
    uint64_t bytes = 0;
    unsigned profileFrequency = 0; // Requested profile, before hardware clock division.
};
extern SdState sd;
bool mountSd();
bool unmountSd();
bool formatSdCard(unsigned mode, unsigned frequency, bool fallback, SdState& mounted, std::string& error);
bool recoverSdCard(unsigned mode, unsigned frequency, const SdState& failed, SdState& mounted, std::string& error);
bool remountSdCard(unsigned mode, unsigned frequency, bool fallback, SdState& mounted, std::string& error);
bool startSdMount(std::string& error);
// These recovery helpers are called with stateMutex held. The worker owns SD
// only after every cancelled frame read has released its file handle.
bool requestSdRecovery(const std::string& reason, bool manual = false);
void serviceSdRecovery();
void sdRecoveryAttempt(unsigned width, unsigned frequency);
cJSON* sdRecoveryJson();
cJSON* sdIoDiagnosticsJson();
bool startSdTool(bool format, unsigned mebibytes, std::string& error,
                 const SdTimingOptions& timing = {});
bool startSdReadTest(unsigned mebibytes, const SdReadTestOptions& options, std::string& error);
bool startSdScope(const SdScopeOptions& options, std::string& error);
bool detachSdForScope(std::string& error);
cJSON* sdScopeJson(); // Caller holds stateMutex; samples appear after capture/restoration.
esp_err_t configureSdReadTiming(unsigned phase, bool continuousClock);
esp_err_t restoreSdReadTiming();
bool sdToolRunning();
void cancelSdSpeedTest();
cJSON* sdToolJson(); // Caller holds stateMutex.
bool sdPath(const std::string& relative, std::string& result);
void installSdFirmware();
esp_err_t installFirmware(FILE* file, size_t size);

struct OutputStats {
    bool lastPulse = false, lastSkipped = false;
    uint32_t lastUs = 0, minUs = 0, maxUs = 0, bits = 0;
    uint32_t lastPackUs = 0, lastCompletionUs = 0, maxCompletionUs = 0;
    uint64_t totalUs = 0, count = 0, totalPackUs = 0, totalCompletionUs = 0;
    uint64_t colorCount = 0, blackCount = 0;
};
struct DmaOutput;
class Output {
public:
    bool begin(unsigned pixels);
    void pixel(unsigned arm, unsigned index, uint8_t r, uint8_t g, uint8_t b, bool applyCenterFade = true, bool tipFirst = false);
    void row(unsigned arm, const uint8_t* colors, bool tipFirst);
    void clear();
    bool show(uint8_t brightness, bool pulse = false, int64_t deadline = 0);
    bool preparePulse(unsigned colorFrames = 1);
    bool signalLevel(int pin, bool high);
    bool enableDma(uint32_t clockHz);
    bool disableDma();
    bool dmaEnabled() const { return dma_ != nullptr; }
    bool ready() const { return rgb_ != nullptr && !fault_; }
    esp_err_t lastError() const { return error_; }
    OutputStats stats;
private:
    uint8_t* rgb_ = nullptr;
    bool hasColor_ = false;
    unsigned pixels_ = 0;
    RadialGainCache<MaxPixels> radialGains_;
    SharedClockProtocol::Protocol protocol_ = SharedClockProtocol::Protocol::Sk9822;
    DmaOutput* dma_ = nullptr;
    bool fault_ = false;
    esp_err_t error_ = ESP_OK;
};
extern Output output;

// Called with stateMutex held; the display task performs one bounded sample.
bool startDmaTest(uint32_t clockHz, std::string& error);
void stepDmaTest();
void cancelDmaTest();
bool dmaTestRunning();
cJSON* dmaTestJson();
bool startStripSpeedTest(uint32_t clockHz, unsigned seconds, std::string& error);
void stepStripSpeedTest();
void cancelStripSpeedTest();
bool stripSpeedTestRunning();
cJSON* stripSpeedTestJson();

enum class Mode { Stopped, Playback, Arms, Hall, Connectors, DmaTest, ColorFade, Solid, SignalCheck,
                  QuarterColors, AlternatingSpokes, StripSpeed, ArmOrder, WhiteBlink, Alignment };
inline bool usesSpokeTiming(Mode mode) {
    return mode == Mode::Playback || mode == Mode::QuarterColors || mode == Mode::AlternatingSpokes ||
           mode == Mode::Alignment;
}
inline bool usesDisplayDuty(Mode mode) {
    return mode == Mode::Playback || mode == Mode::QuarterColors || mode == Mode::AlternatingSpokes;
}
struct Runtime {
    Mode mode = Mode::Stopped;
    bool paused = false, background = false, maintenance = false;
    bool playbackComplete = false;
    uint32_t frame = 0, framesRead = 0, missedSpokes = 0;
    uint32_t lastFrameLoadUs = 0, maxFrameLoadUs = 0, skippedPaints = 0, sdStalls = 0;
    uint32_t maxBlankStartLateUs = 0;
    uint32_t lastPaintPrepareUs = 0, maxPaintPrepareUs = 0;
    int64_t flashProofUntil = 0;
    uint32_t flashProofBursts = 0, flashProofSkipped = 0;
    unsigned flashProofColorFrames = 2;
    std::string flashProofState = "idle";
    int64_t frameLoadStarted = 0;
    int64_t playbackFinishDue = 0;
    bool timingLimited = false;
    uint64_t playbackSession = 0;
    uint16_t spoke = 0;
    uint64_t generation = 0;
    int64_t lastActivity = 0;
    int64_t diagnosticStarted = 0;
    std::array<uint8_t, 3> testColor{255, 0, 0};
    int signalPin = BoardPins::Clock;
    uint8_t signalPattern = 2; // 0=LOW, 1=HIGH, 2=one cycle per second
    bool signalHigh = false;
    bool whiteBlinkOn = false;
    uint32_t whiteBlinkTransitions = 0;
    std::string error;
    // Retain the last successfully opened file's dimensions for rotation tests.
    // Manual settings and these per-file values are kept separate.
    FseqHeader fileHeader;
    std::vector<SparseRange> fileRanges;
    std::string filePath;
    bool sequenceTipFirst = false;
};
extern Runtime runtime;
struct HallSnapshot {
    int64_t last = 0, period = 0, lastPeriod = 0, lastRejectedInterval = 0;
    uint32_t count = 0, rejected = 0;
    unsigned samples = 0;
};
HallSnapshot hallSnapshot();
float rpm();
DutyCalculation dutyCalculation(); // Caller holds stateMutex.
FseqSettings fileSettings(); // Caller holds stateMutex.
unsigned displaySpokes();
int64_t sequenceFrameUs();
void notifyDisplay();
void stopPlayback();
bool startPlayback(const std::string& path, std::string& error);
void startDiagnostic(Mode mode);
bool startSpokeTest(Mode mode, std::string& error);
bool startFlashProof(std::string& error, unsigned colorFrames = 0);
void stopFlashProof(bool complete = false);
bool flashProofRunning();
bool reconfigureOutput();
void restartSoon();
void startNetwork();
void reconnectNetwork();
// Blocking worker calls; never hold stateMutex while waiting for the network task.
bool pauseWifiForScope(std::string& error);
bool resumeWifiAfterScope(std::string& error);
bool wifiQuietForScope();
void addNetworkStatus(cJSON* object);
bool startWifiScan(std::string& error);
cJSON* wifiScanJson();
std::string stationIp();
void startWeb();
cJSON* statusJson();
}
