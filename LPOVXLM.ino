// POV Spinner: rebuilt 16-connector PCB, initial four independent arms.
// ESP32-S3 N16R8: QIO flash + OPI PSRAM; see BoardPins.h and PCB_PIN_REVIEW.md.
// Tim Nash (Inventor)
// All active strips are updated together on the common clock. No chained arms.

#include "ConfigTypes.h"
#include "BoardPins.h"
#include "SharedClockOutput.h"
#include "SharedClockProtocol.h"
#include <esp_heap_caps.h>
#include <Arduino.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <math.h> // fabsf

#include "QuadMap.h"
#include "WebPages.h"
#include "HtmlUtils.h"
#include "WifiManager.h"
#include "SD_Functions.h"
#include "DebugLog.h"

// Toggle physical serial output via USB/COM port. When false, logs are only
// accessible through the Wi-Fi log viewer.
static constexpr bool kEnableSerialDebug = false;

// ---------- Optional zlib backends (auto-detect) ----------
#if defined(__has_include)
  #if __has_include(<miniz.h>)
    #include <miniz.h>
  #endif
  #if __has_include(<zlib.h>)
    #include <zlib.h>
  #endif
#endif

// ---------- FreeRTOS mutex for SD serialization ----------
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ---------- SK9822 / APA102 ----------

// The legacy outmode setting remains in backups; this PCB requires parallel output.
enum OutputMode : uint8_t { OUT_SPI = 0, OUT_PARALLEL = 1 };
uint8_t g_outputMode = OUT_PARALLEL;
static SharedClockOutput g_output;
static_assert(MAX_ARMS == BoardPins::InitialArms, "Update the arm routing before expanding");

// ---------- Hall effect (GPIO48 is now an optional strip output) ----------
static const int      PIN_HALL_SENSOR        = BoardPins::Hall;   // A3144 on this pin (LOW when magnet present)
static bool              g_hallDiagEnabled  = false;
static bool              g_hallDiagActive = false;
static bool              g_armTestEnabled   = false;
static uint8_t           g_armTestCurrentArm = 0;
static uint8_t           g_armTestColorIdx   = 0;
static uint16_t          g_armTestCurrentPixel = 0;
static uint32_t          g_armTestNextStepMs = 0;
static const uint8_t     ARM_TEST_COLORS[3][3] = {
  {255,   0,   0},
  {0,   255,   0},
  {0,     0, 255},
};
static const uint8_t     ARM_TEST_COLOR_COUNT = 3;
static const uint32_t    ARM_TEST_SWEEP_TOTAL_MS = 600;
static const uint32_t    ARM_TEST_STEP_MIN_MS   = 15;
static const uint32_t    ARM_TEST_SWEEP_HOLD_MS = 300;

// Dual-core architecture
static TaskHandle_t g_displayTaskHandle = nullptr;
static SemaphoreHandle_t g_frameMutex = nullptr;
static volatile bool g_displayThreadRunning = false;

// RPM measurement (A3144)
static const uint8_t     PULSES_PER_REV      = BoardPins::PulsesPerRevolution;   // default; can override at runtime via /rpm
volatile uint32_t        g_lastPeriodUs      = 0;   // last valid pulse period (us)
volatile uint32_t        g_pulseCount        = 0;   // total pulses seen
volatile uint32_t        g_lastPulseUsIsr    = 0;   // last pulse timestamp (us) in ISR
static uint32_t          g_rpmUi             = 0;   // filtered RPM for UI/status
static uint32_t          g_lastRpmUpdateMs   = 0;   // ms timestamp of last RPM activity
static const uint8_t     RPM_AVERAGE_REVS    = 2;   // revolutions to average per measurement

// --- NEW: RPM configuration & counting-based measurement ---
static volatile uint8_t  g_pulsesPerRev   = PULSES_PER_REV; // runtime-configurable PPR
// 0=FALLING, 1=RISING, 2=CHANGE
static uint8_t  g_hallEdgeMode   = 0;

static uint32_t g_rpmSampleUs    = 0;  // last sample timestamp (us)
static uint32_t g_rpmLastCount   = 0;  // pulse count snapshot at last sample
static uint64_t g_rpmAccumulatedUs   = 0;  // total us accumulated for current window
static uint32_t g_rpmAccumulatedPulses = 0; // pulses accumulated for current window

// --- Hall sync flags used by ISR and main loop ---
static volatile bool     g_hallSyncPending     = false;
static volatile uint32_t g_hallSyncTimestampUs = 0;

static void IRAM_ATTR hallIsr() {
  uint32_t now = micros();
  uint32_t last = g_lastPulseUsIsr;
  g_lastPulseUsIsr = now;
  // crude debounce: ignore pulses < 1ms apart (spurious)
  uint32_t dt = now - last;
  if (dt > 1000) {
    g_lastPeriodUs = dt;
    g_pulseCount = g_pulseCount + 1;
    uint32_t count = g_pulseCount;
    uint8_t ppr = g_pulsesPerRev ? g_pulsesPerRev : 1;
    if ((ppr > 0) && (count % ppr) == 0) {
      g_hallSyncTimestampUs = now;
      g_hallSyncPending = true;
    }
  }
}

// Allow runtime selection of ISR edge
static void attachHallInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR));
  int mode = (g_hallEdgeMode == 1) ? RISING : (g_hallEdgeMode == 2 ? CHANGE : FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR), hallIsr, mode);
}

extern uint16_t g_pixelsPerArm;
uint8_t g_brightness = 63;
static volatile bool g_needShow = false;

// ---- Arm runtime state (moved up so lanesCommit can see it) ----
struct ArmRuntimeState {
  uint16_t baseSpoke = 0;
  uint16_t currentSpoke = 0;
  uint32_t blankDeadlineUs = 0;
  uint32_t paintTimestampUs = 0;
  bool     lit = false;
};

static ArmRuntimeState g_armState[MAX_ARMS];

static inline void lanesCommit() {
  if (!g_needShow) return;
  g_output.show();
  g_needShow = false;
}

static inline uint16_t armPixelCount() {
  return g_pixelsPerArm ? g_pixelsPerArm : DEFAULT_PIXELS_PER_ARM;
}

static inline void armSetPixel(uint8_t arm, uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
  if (arm >= MAX_ARMS || pixel >= armPixelCount()) return;
  if (BoardPins::ArmReverse[arm]) pixel = armPixelCount() - 1 - pixel;
  g_output.setPixel(arm, pixel, r, g, b);
}

static inline void armShow(uint8_t /*arm*/) { g_needShow = true; }
static inline void lanesShowAll() { g_output.show(); g_needShow = false; }
static inline void armClear(uint8_t arm) { g_output.clear(arm); g_needShow = true; }
static inline void armFillColor(uint8_t arm, uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < armPixelCount(); ++i) armSetPixel(arm, i, r, g, b);
}

// ---------- Watchdog ----------
static const uint32_t WATCHDOG_TIMEOUT_SECONDS = 8;
bool     g_watchdogEnabled   = false;
static bool g_watchdogAttached = false;

static void applyWatchdogSetting() {
  const uint32_t ALL_CORES_MASK = (1U << portNUM_PROCESSORS) - 1U;

  if (g_watchdogEnabled) {
    esp_task_wdt_config_t cfg = {
      .timeout_ms    = WATCHDOG_TIMEOUT_SECONDS * 1000U,
      .idle_core_mask= ALL_CORES_MASK,
      .trigger_panic = true
    };
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_reconfigure(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      DebugLog::printf("[WDT] init/reconfig failed: %d\n", (int)err);
      return;
    }
    if (!g_watchdogAttached) {
      err = esp_task_wdt_add(nullptr);
      if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        g_watchdogAttached = true;
        esp_task_wdt_reset();
        DebugLog::printf("[WDT] Enabled (timeout %us)\n", (unsigned)WATCHDOG_TIMEOUT_SECONDS);
      } else {
        DebugLog::printf("[WDT] add failed: %d\n", (int)err);
      }
    }
  } else if (g_watchdogAttached) {
    esp_task_wdt_delete(nullptr);
    esp_task_wdt_deinit();
    g_watchdogAttached = false;
    DebugLog::println("[WDT] Disabled");
  }
}
void feedWatchdog() { if (g_watchdogAttached) esp_task_wdt_reset(); }

// Persistent scratch for zlib frames
static uint8_t* s_ctmp = nullptr;
static size_t   s_ctmp_size = 0;

// === Quadrant mapping controls ===
static int START_SPOKE_1BASED = 1;
static SpokeLabelMode gLabelMode = FLOOR_TO_BOUNDARY; // used for logging
static const int SPOKES = 40;

// ---------- Wi-Fi + settings backup ----------
static const char* AP_SSID  = "POV-Spinner";
static const char* AP_PASS  = "POV123456";
static const IPAddress AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);
WebServer server(80);

// (SD state is managed by SD_Functions.*)
String        g_staSsid;
String        g_staPass;
String        g_stationId;
bool          g_staConnecting       = false;
bool          g_staConnected        = false;
uint32_t      g_staConnectStartMs   = 0;

// ---------- Persisted settings (NVS = flash) ----------
Preferences prefs;
uint8_t  g_brightnessPercent = 25;
uint8_t  g_displayDutyPercent = 60;
uint16_t g_fps               = 40;
uint32_t g_framePeriodUs = 25000;  // 40 FPS = 25000 microseconds
bool     g_autoplayEnabled   = true;
bool     g_bgEffectEnabled   = false;
bool     g_bgEffectActive    = false;
String   g_bgEffectPath;
uint32_t g_bgEffectNextAttemptMs = 0;

// Spinner model mapping (persisted)
uint32_t g_startChArm1   = 1;    // 1-based absolute channel (R of Arm1, Pixel0)
uint16_t g_spokesTotal   = 40;
uint8_t  g_armCount      = MAX_ARMS;
uint16_t g_pixelsPerArm  = DEFAULT_PIXELS_PER_ARM;
static uint16_t g_lastPulseSpoke[MAX_ARMS] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

// ================= Per-arm start channel mapping (optional) =================
bool     g_usePerArmStart = false;            // NVS key: "usepa"
uint32_t g_startChArm[MAX_ARMS] = {0,0,0,0};  // 1-based absolute channel for Arm1..Arm4 (R)
static void computeDefaultArmStarts(uint32_t startArm1); // fwd decl

// Rotary index (0..spokes-1)
volatile uint16_t g_indexPosition = 0;

// Playback state
volatile bool  g_playing = false, g_paused = false;
String         g_currentPath;
uint32_t       g_frameIndex = 0, g_lastTickUs = 0;
uint32_t       g_bootMs = 0;
const uint32_t SELECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;

static uint32_t        g_spokeDurationUs     = 0;
static uint32_t        g_nextSpokeDeadlineUs = 0;
static uint16_t        g_spokeStep           = 0;
static const uint32_t  ARM_BLANK_MIN_US      = 40;   // minimum ON microseconds per spoke
static const uint32_t  ARM_BLANK_FALLBACK_US = 1000; // fallback if duration unknown
static bool            g_frameValid          = false;

// Also ADD these for diagnostics:
uint32_t g_frameCounter = 0;
uint32_t g_lastFpsReportUs = 0;
float    g_measuredFps = 0.0f;

static inline bool microsReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

static void resetArmRuntimeStates();
static void blankArm(uint8_t arm);
static void paintArmAt(uint8_t arm, uint16_t spokeIdx, uint32_t nowUs);
static uint32_t computeArmHoldDurationUs();
static void processArmBlanking(uint32_t nowUs);
static void processHallSyncEvent(uint32_t nowUs);
static void advancePredictedSpokes(uint32_t nowUs);
static bool loadNextFrame();

/* -------------------- Strobe gating / angular timing -------------------- */
static const int PIN_STROBE_GATE = -1; // -1 to disable gate pin

static volatile bool  g_strobeEnable   = false;   // DEFAULT NOW OFF
static volatile float g_strobeWidthDeg = 3.0f;
static volatile float g_strobePhaseDeg = 0.0f;
static float g_armPhaseDeg[MAX_ARMS] = {0.0f, 0.0f, 0.0f, 0.0f};

static inline uint16_t spokesCount() { return (g_spokesTotal ? g_spokesTotal : 1); }

static inline void getHallSnapshot(uint32_t& periodUs, uint32_t& sinceUs) {
  uint32_t lastIsr = g_lastPulseUsIsr;
  periodUs = g_lastPeriodUs;
  uint32_t now = micros();
  sinceUs = now - lastIsr;
}

static inline float sinceUsToDeg(uint32_t sinceUs, uint32_t periodUs) {
  if (periodUs == 0) return 0.0f;
  return (360.0f * (float)sinceUs) / (float)periodUs;
}

static inline uint16_t currentSpokeIndex() {
  uint32_t perUs, sinceUs; getHallSnapshot(perUs, sinceUs);
  if (perUs == 0) return g_indexPosition;
  float deg = sinceUsToDeg(sinceUs, perUs) + g_strobePhaseDeg;
  while (deg < 0.0f)  deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  const uint16_t s = spokesCount();
  uint32_t idx = (uint32_t)((deg / 360.0f) * s);
  if (idx >= s) idx = s - 1;
  return (uint16_t)idx;
}

static inline bool inStrobeWindowForArm(uint16_t spokeCenter, uint8_t arm) {
  if (!g_strobeEnable) return true;
  uint32_t perUs, sinceUs; getHallSnapshot(perUs, sinceUs);
  if (perUs == 0) return true;
  float deg = sinceUsToDeg(sinceUs, perUs) + g_armPhaseDeg[arm] + g_strobePhaseDeg;
  while (deg < 0.0f)  deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  const float spokeDegW = 360.0f / (float)spokesCount();
  const float centerDeg = (spokeCenter + 0.5f) * spokeDegW;
  float delta = fabsf(deg - centerDeg);
  if (delta > 180.0f) delta = 360.0f - delta;

  return (delta <= (g_strobeWidthDeg * 0.5f));
}

static void setDefaultArmPhases() {
  const uint8_t arms = (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount);
  bool allZero = true;
  for (uint8_t a = 0; a < arms; ++a) {
    if (fabsf(g_armPhaseDeg[a]) > 1e-3f) { allZero = false; break; }
  }
  if (!allZero) return;

  const float sep = 360.0f / (float)arms;
  for (uint8_t a = 0; a < arms; ++a) {
    g_armPhaseDeg[a] = a * sep;
  }
}

/* -------------------- Hall effect handling (blink + diag) -------------------- */
static void updateHallSensor() {
  const bool hallActive = (digitalRead(PIN_HALL_SENSOR) == LOW);

  if (g_hallDiagEnabled) {
    if (hallActive && !g_hallDiagActive) {
      g_hallDiagActive = true;
      const uint8_t arms = (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount);
      // Fill all arms red and transmit them together.
      for (uint8_t a = 0; a < arms; ++a) {
        const uint16_t n = armPixelCount();
        for (uint16_t i = 0; i < n; ++i) armSetPixel(a, i, 255, 0, 0);
      }
      lanesShowAll();
    } else if (!hallActive && g_hallDiagActive) {
      const uint8_t arms = (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount);
      for (uint8_t a = 0; a < arms; ++a) {
        const uint16_t n = armPixelCount();
        for (uint16_t i = 0; i < n; ++i) armSetPixel(a, i, 0, 0, 0);
      }
      lanesShowAll();
      g_hallDiagActive = false;
    }
  } else if (g_hallDiagActive) {
    const uint8_t arms = (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount);
    for (uint8_t a = 0; a < arms; ++a) {
      const uint16_t n = armPixelCount();
      for (uint16_t i = 0; i < n; ++i) armSetPixel(a, i, 0, 0, 0);
    }
    lanesShowAll();
    g_hallDiagActive = false;
  }

}

static void updateArmTest() {
  if (!g_armTestEnabled) return;

  const uint8_t arms = activeArmCount();
  if (!arms) return;
  const uint16_t pixels = armPixelCount();
  if (!pixels) return;

  uint32_t now = millis();
  if (g_armTestNextStepMs && now < g_armTestNextStepMs) return;

  if (g_armTestCurrentArm >= arms) g_armTestCurrentArm = 0;
  if (g_armTestColorIdx >= ARM_TEST_COLOR_COUNT) g_armTestColorIdx = 0;
  if (g_armTestCurrentPixel >= pixels) g_armTestCurrentPixel = 0;

  const uint8_t *color = ARM_TEST_COLORS[g_armTestColorIdx];
  for (uint8_t a = 0; a < arms; ++a) armFillColor(a, 0, 0, 0);

  if (g_armTestCurrentArm < arms) {
    uint16_t maxPixel = g_armTestCurrentPixel;
    if (maxPixel >= pixels) maxPixel = pixels - 1;
    for (uint16_t p = 0; p <= maxPixel; ++p) {
      armSetPixel(g_armTestCurrentArm, p, color[0], color[1], color[2]);
    }
  }

  lanesShowAll();

  uint32_t stepMs = (pixels > 0) ? (ARM_TEST_SWEEP_TOTAL_MS / pixels) : ARM_TEST_SWEEP_TOTAL_MS;
  if (stepMs < ARM_TEST_STEP_MIN_MS) stepMs = ARM_TEST_STEP_MIN_MS;

  ++g_armTestCurrentPixel;
  if (g_armTestCurrentPixel >= pixels) {
    g_armTestCurrentPixel = 0;
    ++g_armTestCurrentArm;
    stepMs = ARM_TEST_SWEEP_HOLD_MS;
    if (g_armTestCurrentArm >= arms) {
      g_armTestCurrentArm = 0;
      g_armTestColorIdx = (g_armTestColorIdx + 1) % ARM_TEST_COLOR_COUNT;
    }
  }

  g_armTestNextStepMs = now + stepMs;
}

/* -------------------- Web handlers (decls) -------------------- */
static void handleStatus();
static void handleRoot();
static void handleB();
static void handleStart();
static void handleStop();
static void handlePause();
static void handleHallDiag();
static void handleArmTest();
static void handleSpeed();
static void handleMapCfg();
static void handleWifiCfg();
static void handleFseqHeader();
static void handleCBlocks();
static void handleAutoplay();
static void handleWatchdog();
static void handleBgEffect();
static void handleStrobe();
static void handleArmPhase();
static void handleRpmCfg();
static void handleReboot();
static void handleOutMode(); // declared here; implemented later with setOutputMode()
static void handleDiagMap();     // /diag/map?arm=1&pix=0&spoke=0
static void handleFseqRanges();  // /fseq/ranges
static void handleLaneDiag();    // /lanediag
static void handleLogsPage();
static void handleLogsText();
static void handleLogsClear();

static bool otaAuthOK() { return true; } // stub (shared with SD module)

/* -------------------- FSEQ v2 reader -------------------- */
struct SparseRange { uint32_t start, count, accum; };
struct CompBlock    { uint32_t uSize, cSize; };

struct FseqHeader {
  uint16_t chanDataOffset = 0;
  uint8_t  minor = 0, major = 2;
  uint16_t varDataOffset = 0;
  uint32_t channelCount = 0;
  uint32_t frameCount   = 0;
  uint8_t  stepTimeMs   = 25;
  uint8_t  flags        = 0;
  uint8_t  compType     = 0;     // 0=none, 1=zstd, 2=zlib
  uint8_t  compBlockCnt = 0;
  uint8_t  sparseCnt    = 0;
  uint64_t uniqueId     = 0;
};

File         g_fseq;
FseqHeader   g_fh;
SparseRange* g_ranges      = nullptr;
uint8_t*     g_frameBuf    = nullptr;
CompBlock*   g_cblocks     = nullptr;
uint32_t     g_compCount   = 0;
uint64_t     g_compBase    = 0;
bool         g_compPerFrame= false;

/* -------------------- Small helpers -------------------- */
static inline int32_t  clampI32(int32_t v, int32_t lo, int32_t hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static inline uint8_t  activeArmCount(){ return (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount); }

// --- Binary read helpers (little-endian) ---
static inline bool readU16(File &f, uint16_t &out) {
  uint8_t b[2]; if (f.read(b,2)!=2) return false;
  out = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  return true;
}
static inline bool readU32(File &f, uint32_t &out) {
  uint8_t b[4]; if (f.read(b,4)!=4) return false;
  out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  return true;
}
static inline bool readU64(File &f, uint64_t &out) {
  uint8_t b[8]; if (f.read(b,8)!=8) return false;
  out =  ((uint64_t)b[0])        | ((uint64_t)b[1] << 8)  | ((uint64_t)b[2] << 16) | ((uint64_t)b[3] << 24)
       | ((uint64_t)b[4] << 32)  | ((uint64_t)b[5] << 40) | ((uint64_t)b[6] << 48) | ((uint64_t)b[7] << 56);
  return true;
}

/* -------------------- FSEQ open/close/load -------------------- */
static void freeFseq(){
  if (g_fseq) g_fseq.close();
  if (g_ranges){ free(g_ranges); g_ranges=nullptr; }
  if (g_frameBuf){ free(g_frameBuf); g_frameBuf=nullptr; }
  if (g_cblocks){ free(g_cblocks); g_cblocks=nullptr; }
  if (s_ctmp){ free(s_ctmp); s_ctmp=nullptr; s_ctmp_size=0; }
  g_compCount=0; g_compBase=0; g_compPerFrame=false;
  g_bgEffectActive = false;
  memset(&g_fh,0,sizeof(g_fh));
  g_frameValid = false;
  g_frameIndex = 0;
  resetArmRuntimeStates();
  g_playing = false;
  g_paused = false;
  g_currentPath = "";
}

static int64_t sparseTranslate(uint32_t absCh) {
  if (g_fh.sparseCnt == 0) {
    return (absCh < g_fh.channelCount) ? (int64_t)absCh : -1;
  }
  for (uint8_t i=0;i<g_fh.sparseCnt;++i){
    const SparseRange &r = g_ranges[i];
    if (absCh >= r.start && absCh < r.start + r.count) {
      return (int64_t)r.accum + (absCh - r.start);
    }
  }
  return -1;
}

bool openFseq(const String& path, String& why){
  freeFseq();
  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { why="sd busy"; return false; }
  bool ok = false;
  do {
    g_fseq = SD_MMC.open(path, FILE_READ);
    if (!g_fseq){ why="open fail"; break; }

    uint8_t magic[4]; if (g_fseq.read(magic,4)!=4){ why="short"; break; }
    if (!((magic[0]=='F'||magic[0]=='P') && magic[1]=='S' && magic[2]=='E' && magic[3]=='Q')){ why="bad magic"; break; }

    if (!readU16(g_fseq, g_fh.chanDataOffset) || g_fseq.read(&g_fh.minor,1)!=1 || g_fseq.read(&g_fh.major,1)!=1 ||
        !readU16(g_fseq, g_fh.varDataOffset) || !readU32(g_fseq, g_fh.channelCount) ||
        !readU32(g_fseq, g_fh.frameCount) || g_fseq.read(&g_fh.stepTimeMs,1)!=1 || g_fseq.read(&g_fh.flags,1)!=1) { why="hdr"; break; }

    uint8_t ecct_ct_scc_res[4]; if (g_fseq.read(ecct_ct_scc_res,4)!=4){ why="hdr"; break; }
    g_fh.compType     = (ecct_ct_scc_res[0] & 0x0F);
    g_fh.compBlockCnt = ecct_ct_scc_res[1];
    g_fh.sparseCnt    = ecct_ct_scc_res[2];
    if (!readU64(g_fseq, g_fh.uniqueId)) { why="hdr"; break; }

    if (g_fh.compBlockCnt > 0) {
      g_cblocks = (CompBlock*)malloc(sizeof(CompBlock)*g_fh.compBlockCnt);
      if (!g_cblocks){ why="oom ctab"; break; }
      for (uint32_t i=0;i<g_fh.compBlockCnt;++i){
        if (!readU32(g_fseq, g_cblocks[i].uSize) || !readU32(g_fseq, g_cblocks[i].cSize)) { why="ctab"; break; }
      }
      g_compCount = g_fh.compBlockCnt;
    }

    if (g_fh.sparseCnt > 0){
      g_ranges = (SparseRange*)malloc(sizeof(SparseRange)*g_fh.sparseCnt);
      if (!g_ranges){ why="oom ranges"; break; }
      uint32_t accum=0;
      for (uint8_t i=0;i<g_fh.sparseCnt;++i){
        uint8_t b[6]; if (g_fseq.read(b,6)!=6){ why="ranges"; break; }
        uint32_t start = (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16);
        uint32_t count = (uint32_t)b[3]|((uint32_t)b[4]<<8)|((uint32_t)b[5]<<16);
        g_ranges[i] = { start, count, accum };
        accum += count;
      }
    }

    g_fseq.seek(g_fh.chanDataOffset, SeekSet);

    if (g_fh.compType == 0) {
      g_compBase = g_fh.chanDataOffset;
    } else if (g_fh.compType == 2) {
      g_compBase = g_fh.chanDataOffset;
      bool perFrame = (g_compCount == g_fh.frameCount && g_fh.channelCount>0);
      if (perFrame){ for (uint32_t i=0;i<g_compCount;++i){ if (g_cblocks[i].uSize != g_fh.channelCount){ perFrame=false; break; } } }
#if defined(MZ_OK) || defined(Z_OK)
      g_compPerFrame = perFrame;
      if (!g_compPerFrame) { why="zlib block!=frame (not yet supported)"; break; }
#else
      (void)perFrame; why="zlib not available"; break;
#endif
    } else { why="zstd unsupported"; break; }

    if (g_fh.channelCount==0){ why="zero chans"; break; }
    g_frameBuf = (uint8_t*)heap_caps_malloc(g_fh.channelCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_frameBuf) g_frameBuf = (uint8_t*)heap_caps_malloc(g_fh.channelCount, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!g_frameBuf){ why="oom frame"; break; }

    g_currentPath = path;
    g_frameIndex = 0;
    g_bgEffectActive = g_bgEffectEnabled && isBgEffectPath(g_currentPath);

    ok = true;
  } while(0);
  SD_UNLOCK();

  if (ok) {
    resetArmRuntimeStates();
    g_frameValid = false;
    if (!loadNextFrame()) { why = "frame load"; ok = false; }
    else {
      g_lastTickUs = micros();  // CHANGE from g_lastTickMs = millis()
      g_playing = true;
      g_paused = false;
      DebugLog::printf("[FSEQ] %s frames=%lu chans=%lu step=%ums comp=%u blocks=%u sparse=%u CDO=0x%04x\n",
        path.c_str(), (unsigned long)g_fh.frameCount, (unsigned long)g_fh.channelCount,
        g_fh.stepTimeMs, g_fh.compType, (unsigned)g_compCount, (unsigned)g_fh.sparseCnt, g_fh.chanDataOffset);
    }
  }

  if (!ok) { freeFseq(); }

  return ok;
}

static bool loadFrame(uint32_t idx){
  if (!g_fseq || !g_fh.frameCount) return false;
  idx %= g_fh.frameCount;

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) return false;
  bool ok=false;

  if (g_fh.compType == 0){
    const uint64_t base = (uint64_t)g_fh.chanDataOffset + (uint64_t)idx * (uint64_t)g_fh.channelCount;
    if (g_fseq.seek(base, SeekSet))
      ok = (g_fseq.read(g_frameBuf, g_fh.channelCount) == g_fh.channelCount);
  }
#if defined(MZ_OK) || defined(Z_OK)
  else if (g_fh.compType == 2 && g_compPerFrame){
    uint64_t offs = g_compBase;
    for (uint32_t i=0;i<idx;++i) offs += g_cblocks[i].cSize;
    if (g_fseq.seek(offs, SeekSet)) {
      uint32_t clen = g_cblocks[idx].cSize;
      if (clen && clen <= 8*1024*1024) {
        if (s_ctmp_size < clen) {
          uint8_t* nb = (uint8_t*)heap_caps_realloc(s_ctmp, clen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
          if (!nb) nb = (uint8_t*)heap_caps_realloc(s_ctmp, clen, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (!nb) { SD_UNLOCK(); return false; }
          s_ctmp = nb; s_ctmp_size = clen;
        }
        size_t got = g_fseq.read(s_ctmp, clen);
        if (got == clen) ok = zlib_decompress(s_ctmp, clen, g_frameBuf, g_fh.channelCount);
      }
    }
  }
#endif
  SD_UNLOCK();
  return ok;
}

static bool loadNextFrame(){
  if (!g_fseq || !g_fh.frameCount) return false;
  uint32_t count = g_fh.frameCount;
  uint32_t idx = g_frameIndex % count;
  if (!loadFrame(idx)) return false;
  g_frameIndex = (idx + 1) % count;
  g_frameValid = true;
  return true;
}

/* -------------------- Color mapping -------------------- */
enum ColorMap { MAP_RGB, MAP_RBG, MAP_GBR, MAP_GRB, MAP_BRG, MAP_BGR };
ColorMap g_colorMap = MAP_RGB;

static inline void mapChannels(const uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b) {
  switch (g_colorMap) {
    case MAP_RGB: r=p[0]; g=p[1]; b=p[2]; break;
    case MAP_RBG: r=p[0]; b=p[1]; g=p[2]; break;
    case MAP_GBR: g=p[0]; b=p[1]; r=p[2]; break;
    case MAP_GRB: g=p[0]; r=p[1]; b=p[2]; break;
    case MAP_BRG: b=p[0]; r=p[1]; g=p[2]; break;
    case MAP_BGR: b=p[0]; g=p[1]; r=p[2]; break;
  }
}

// Compute per-arm starting channels from g_startChArm1.
static void computeDefaultArmStarts(uint32_t startArm1) {
  if (startArm1 < 1) startArm1 = 1;

  const uint8_t arms = activeArmCount();
  const uint16_t pix = armPixelCount();

  for (uint8_t a = 0; a < MAX_ARMS; ++a) g_startChArm[a] = 0;

  for (uint8_t a = 0; a < arms; ++a) {
    g_startChArm[a] = startArm1 + (uint32_t)a * (uint32_t)pix * 3u;
  }
}

/* -------------------- Rebuild four independent strips -------------------- */
static void rebuildStrips() {
  // Clock low before parking the twelve unused data inputs. No memory pins
  // are touched, even if preferences came from the old GPIO35/36 build.
  static bool parked = false;
  if (!parked) {
    digitalWrite(BoardPins::Clock, LOW);
    pinMode(BoardPins::Clock, OUTPUT);
    for (int pin : BoardPins::OutputData) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
    }
    parked = true;
  }
  g_output.setBrightness(g_brightness);
  if (!g_output.begin(BoardPins::Clock, BoardPins::ArmData, MAX_ARMS, armPixelCount())) {
    DebugLog::println("[OUTPUT] Initialization failed; playback disabled");
    g_playing = false;
    return;
  }
  for (uint8_t arm = 0; arm < MAX_ARMS; ++arm) {
    DebugLog::printf("[ARM%u] connector 16-%u: DATA=%d CLK=%d pixels=%u reverse=%d\n",
                     arm + 1, BoardPins::ActiveConnectors[arm], BoardPins::ArmData[arm],
                     BoardPins::Clock, armPixelCount(), BoardPins::ArmReverse[arm]);
  }
  resetArmRuntimeStates();
  setDefaultArmPhases();
}

/* -------------------- Arm runtime / blanking -------------------- */
static void resetArmRuntimeStates(){
  noInterrupts();
  g_hallSyncPending = false;
  g_hallSyncTimestampUs = 0;
  interrupts();
  for (uint8_t a=0; a<MAX_ARMS; ++a) {
    g_armState[a].baseSpoke = 0;
    g_armState[a].currentSpoke = 0;
    g_armState[a].blankDeadlineUs = 0;
    g_armState[a].lit = false;
    g_lastPulseSpoke[a] = 0xFFFF;
  }
  g_spokeDurationUs = 0;
  g_nextSpokeDeadlineUs = 0;
  g_spokeStep = 0;
}

// Identify the four physical arm connectors by color.
static void handleLaneDiag() {
  g_playing = false; g_paused = false;
  g_hallDiagEnabled = false; g_armTestEnabled = false;
  blackoutAll();

  const uint8_t arms = activeArmCount();
  if (arms >= 1) armFillColor(0, 255,   0,   0); // Arm1 = RED
  if (arms >= 2) armFillColor(1,   0, 255,   0); // Arm2 = GREEN
  if (arms >= 3) armFillColor(2,   0,   0, 255); // Arm3 = BLUE
  if (arms >= 4) armFillColor(3, 255, 255, 255); // Arm4 = WHITE
  lanesShowAll();

  server.send(200, "application/json", "{\"lanediag\":\"shown\"}");
}

/* -------------------- Draw / blank independent arms -------------------- */
static void blackoutAll(){
  for (uint8_t a=0; a<MAX_ARMS; ++a) blankArm(a);
  lanesCommit();
  resetArmRuntimeStates();
}

static uint32_t computeArmHoldDurationUs(){
  uint32_t base = g_spokeDurationUs;
  if (base == 0) {
    const uint16_t spokes = spokesCount();
    if (spokes > 0 && g_lastPeriodUs > 0) {
      uint8_t ppr = g_pulsesPerRev ? g_pulsesPerRev : 1;
      uint64_t revUs = (uint64_t)g_lastPeriodUs * (uint64_t)ppr;
      base = (uint32_t)(revUs / (uint64_t)spokes);
    }
  }
  if (base == 0) base = ARM_BLANK_FALLBACK_US;

  uint32_t duty = g_displayDutyPercent;
  if (duty > 100) duty = 100;

  // Calculate hold time based on duty cycle
  uint64_t hold = ((uint64_t)base * (uint64_t)duty) / 100ULL;

  // REMOVED minimum clamp - allow duty to go all the way to 0
  // This allows full range control from 0% (instant blank) to 100% (full spoke)
  // The old ARM_BLANK_MIN_US clamp prevented low duty cycles from working

  return (uint32_t)hold;
}

static void paintArmAt(uint8_t arm, uint16_t spokeIdx, uint32_t nowUs){
  if (arm >= MAX_ARMS) return;

  const uint32_t holdUs = computeArmHoldDurationUs();

  // Each arm keeps its own image/phase; commit all four on the shared clock.
  const uint16_t spokes = spokesCount();
  const uint8_t  arms   = activeArmCount();
  const uint16_t pixelCount = armPixelCount();

  if (!g_frameValid || !g_frameBuf || g_fh.channelCount == 0 || arms == 0 || pixelCount == 0) {
    blankArm(arm);
    return;
  }

  // Typical export: one spoke per frame
  const uint32_t expectedPerSpoke = (uint32_t)arms * (uint32_t)pixelCount * 3u;
  const bool perSpokeFrame = (g_fh.channelCount == expectedPerSpoke);

  uint32_t chPerSpoke = expectedPerSpoke;
  if (!perSpokeFrame) {
    if (spokes > 0 && (g_fh.channelCount % spokes) == 0) {
      chPerSpoke = g_fh.channelCount / spokes;
    }
  }

  // Base (R) channel for this arm, pixel 0 (1-based → 0-based)
  uint32_t baseChAbsR = 0;
  if (g_usePerArmStart) {
    baseChAbsR = (g_startChArm[arm] ? g_startChArm[arm]-1 : 0);
  } else {
    const uint32_t base = (g_startChArm1 ? g_startChArm1-1 : 0);
    const uint32_t armBlockStride = (uint32_t)pixelCount * 3u;
    baseChAbsR = base + (uint32_t)arm * armBlockStride;
  }

  // If a frame carries multiple spokes, add the spoke slice offset
  if (!perSpokeFrame) {
    const uint16_t s = (spokes ? spokes : 1);
    baseChAbsR += ((uint32_t)(spokeIdx % s)) * chPerSpoke;
  }

  if (spokes) spokeIdx %= spokes;
  g_armState[arm].currentSpoke = spokeIdx;

  for (uint16_t i = 0; i < pixelCount; ++i) {
    const uint32_t absR = baseChAbsR + (uint32_t)i * 3u;
    const int64_t  idxR = sparseTranslate(absR);

    uint8_t R=0,G=0,B=0;
    if (idxR >= 0 && (idxR + 2) < (int64_t)g_fh.channelCount) {
      mapChannels(&g_frameBuf[idxR], R, G, B);
    }

    /*if (g_brightness < 255) {
      R = (uint8_t)((uint16_t)R * g_brightness / 255);
      G = (uint8_t)((uint16_t)G * g_brightness / 255);
      B = (uint8_t)((uint16_t)B * g_brightness / 255);
    }*/
    armSetPixel(arm, i, R, G, B);
  }

  armShow(arm);
  g_armState[arm].lit = true;
  g_armState[arm].paintTimestampUs = nowUs;  // NEW: store paint time
  g_armState[arm].blankDeadlineUs = nowUs + holdUs;
  if (g_armState[arm].blankDeadlineUs == 0) g_armState[arm].blankDeadlineUs = 1;
}

static void processArmBlanking(uint32_t nowUs){
  const uint8_t arms = activeArmCount();
  bool anyBlanked = false;

  // Add a small lead time to account for processing overhead
  // This ensures we blank ON TIME rather than slightly late
  const uint32_t BLANK_LEAD_US = 20; // blank 20us early to compensate for latency

  for (uint8_t a=0; a<arms; ++a){
    if (!g_armState[a].lit) continue;
    uint32_t blankAt = g_armState[a].blankDeadlineUs;

    // Blank if deadline reached OR if we're within the lead window
    if (blankAt > 0) {
      int32_t timeUntilBlank = (int32_t)(blankAt - nowUs);
      if (timeUntilBlank <= (int32_t)BLANK_LEAD_US) {
        blankArm(a);
        anyBlanked = true;
      }
    }
  }

  for (uint8_t a=arms; a<MAX_ARMS; ++a){
    if (g_armState[a].lit) {
      blankArm(a);
      anyBlanked = true;
    }
  }

  // Commit immediately if anything blanked
  if (anyBlanked) {
    lanesCommit();
  }
}

static void processHallSyncEvent(uint32_t nowUs){
  uint32_t syncUs = 0;
  bool haveSync = false;

  noInterrupts();
  if (g_hallSyncPending) {
    syncUs = g_hallSyncTimestampUs;
    g_hallSyncPending = false;
    haveSync = true;
  }
  interrupts();
  if (!haveSync) return;

  const uint16_t spokes = spokesCount();
  if (!spokes) return;

  const uint8_t arms = activeArmCount();
  const int startIdx0 = spoke1BasedToIdx0(START_SPOKE_1BASED, (int)spokes);

  for (uint8_t a = 0; a < arms; ++a){
    uint16_t base = (uint16_t)armSpokeIdx0((int)a, startIdx0, (int)spokes, (int)arms);
    base %= spokes;
    g_armState[a].baseSpoke = base;

    if (!g_strobeEnable) {
      paintArmAt(a, base, nowUs);
    } else {
      if (g_armState[a].lit) blankArm(a);
      g_lastPulseSpoke[a] = 0xFFFF;
    }
  }
  for (uint8_t a = arms; a < MAX_ARMS; ++a){
    g_armState[a].baseSpoke = 0;
    if (g_armState[a].lit) blankArm(a);
  }

  // Commit initial base paint when strobe is OFF
  if (!g_strobeEnable) lanesCommit();

  uint64_t revolutionUs = 0;
  if (g_lastPeriodUs > 0) {
    uint8_t ppr = g_pulsesPerRev ? g_pulsesPerRev : 1;
    revolutionUs = (uint64_t)g_lastPeriodUs * (uint64_t)ppr;
  } else if (g_spokeDurationUs > 0 && spokes > 0) {
    revolutionUs = (uint64_t)g_spokeDurationUs * (uint64_t)spokes;
  }
  if (revolutionUs == 0) revolutionUs = 1000000ULL;

  uint32_t newDur = (uint32_t)(revolutionUs / (uint64_t)spokes);
  if (newDur == 0) newDur = 1;

  uint32_t prev = g_spokeDurationUs;
  if (prev > 0) {
    newDur = (uint32_t)(((uint64_t)prev * 3ULL + newDur) / 4ULL);
    if (newDur == 0) newDur = 1;
  }

  g_spokeDurationUs = newDur;
  g_spokeStep = 0;
  g_nextSpokeDeadlineUs = syncUs + g_spokeDurationUs;
  if (g_nextSpokeDeadlineUs == 0) g_nextSpokeDeadlineUs = 1;
}

static void advancePredictedSpokes(uint32_t nowUs){
  const uint16_t spokes = spokesCount();
  if (!spokes) return;
  if (g_spokeDurationUs == 0 || g_nextSpokeDeadlineUs == 0) return;

  const uint8_t arms = activeArmCount();
  uint8_t stepsProcessed = 0;
  const uint8_t maxSteps = 4; // Limit steps per loop to avoid blocking

  while (microsReached(nowUs, g_nextSpokeDeadlineUs) && stepsProcessed < maxSteps) {
    g_spokeStep = (g_spokeStep + 1) % spokes;
    for (uint8_t a=0; a<arms; ++a){
      uint16_t base = g_armState[a].baseSpoke % spokes;
      uint16_t spoke = (base + g_spokeStep) % spokes;
      paintArmAt(a, spoke, nowUs);
    }
    g_nextSpokeDeadlineUs += g_spokeDurationUs;
    stepsProcessed++;
  }

  // Commit all painted spokes at once
  if (stepsProcessed > 0) {
    lanesCommit();
  }
}

/* -------------------- Web: Files page + ops -------------------- */
// (unchanged file handlers – implemented in SD_Functions/WebPages modules)

/* -------------------- Web: Control page & API -------------------- */
static inline const char* statusText() { if (g_paused && g_playing) return "Paused"; if (g_playing) return "Playing"; return "Stopped"; }
static inline const char* statusClass(){ if (g_paused && g_playing) return "badge pause"; if (g_playing) return "badge play"; return "badge stop"; }

static uint32_t computeRpmSnapshot() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  uint32_t pulsesPerRev = (g_pulsesPerRev ? g_pulsesPerRev : 1u);
  uint32_t targetPulses = pulsesPerRev * RPM_AVERAGE_REVS;
  if (g_rpmSampleUs == 0) {
    g_rpmSampleUs  = nowUs;
    g_rpmLastCount = g_pulseCount;
    g_rpmAccumulatedUs = 0;
    g_rpmAccumulatedPulses = 0;
    return g_rpmUi;
  }

  uint32_t dtUs = nowUs - g_rpmSampleUs;
  if (dtUs >= 250000) {
    g_rpmSampleUs = nowUs;
    g_rpmAccumulatedUs += dtUs;

    uint32_t countNow = g_pulseCount;
    uint32_t delta    = countNow - g_rpmLastCount;
    if (delta > 0) {
      g_rpmAccumulatedPulses += delta;
      g_rpmLastCount = countNow;
      g_lastRpmUpdateMs = nowMs;
      if (g_rpmAccumulatedPulses >= targetPulses && g_rpmAccumulatedUs > 0) {
        uint64_t denom = (uint64_t)g_rpmAccumulatedUs * (uint64_t)pulsesPerRev;
        uint64_t num   = (uint64_t)g_rpmAccumulatedPulses * 60000000ULL;
        uint32_t inst  = (uint32_t)((num + denom / 2) / denom);
        if (inst > 0) {
          g_rpmUi = (g_rpmUi * 3 + inst) / 4;
        }
        g_rpmAccumulatedUs = 0;
        g_rpmAccumulatedPulses = 0;
      }
    }
  }

  uint32_t staleThresholdMs = 2000;
  if (g_rpmAccumulatedPulses > 0 && g_rpmAccumulatedPulses < targetPulses && g_rpmAccumulatedUs > 0) {
    uint64_t avgPulseUs = g_rpmAccumulatedUs / g_rpmAccumulatedPulses;
    uint64_t expectedUs = avgPulseUs * (uint64_t)(targetPulses - g_rpmAccumulatedPulses);
    uint32_t expectedMs = (uint32_t)(expectedUs / 1000ULL);
    uint32_t bufferedMs = expectedMs + 500u;
    if (bufferedMs > staleThresholdMs) staleThresholdMs = bufferedMs;
  }

  if (nowMs - g_lastRpmUpdateMs > staleThresholdMs) {
    if (g_rpmAccumulatedPulses > 0 && g_rpmAccumulatedUs > 0) {
      uint64_t denom = (uint64_t)g_rpmAccumulatedUs * (uint64_t)pulsesPerRev;
      uint64_t num   = (uint64_t)g_rpmAccumulatedPulses * 60000000ULL;
      uint32_t inst  = (uint32_t)((num + denom / 2) / denom);
      if (inst > 0) {
        g_rpmUi = (g_rpmUi * 3 + inst) / 4;
      }
    } else {
      g_rpmUi = 0;
    }
    g_rpmAccumulatedUs = 0;
    g_rpmAccumulatedPulses = 0;
    g_rpmSampleUs = nowUs;
    g_rpmLastCount = g_pulseCount;
    g_lastRpmUpdateMs = nowMs;
  }
  return g_rpmUi;
}

static void handleStatus(){
  String json = String("{\"playing\":")+(g_playing?"true":"false")
    +",\"paused\":"+(g_paused?"true":"false")
    +",\"path\":\""+htmlEscape(g_currentPath)+"\""
    +",\"frame\":"+String(g_frameIndex)
    +",\"fps\":"+String(g_fps)
    +",\"measuredFps\":"+String(g_measuredFps, 2)
    +",\"displayDuty\":"+String((unsigned)g_displayDutyPercent)
    +",\"startChArm1\":"+String(g_startChArm1)
    +",\"spokes\":"+String(g_spokesTotal)
    +",\"arms\":"+String(g_armCount)
    +",\"pixels\":"+String(g_pixelsPerArm)
    +",\"index\":"+String(g_indexPosition);
  json += ",\"sd\":{\"ready\":";
  json += (g_sdReady?"true":"false");
  json += ",\"currentWidth\":" + String((unsigned)g_sdBusWidth);
  json += ",\"desiredMode\":" + String((unsigned)g_sdPreferredBusWidth);
  json += ",\"baseFreq\":" + String((unsigned long)g_sdBaseFreqKHz);
  json += ",\"freq\":" + String((unsigned long)g_sdFreqKHz);
  json += "}";
  json += ",\"hallDiag\":" + String(g_hallDiagEnabled ? "true" : "false");
  json += ",\"armTest\":" + String(g_armTestEnabled ? "true" : "false");
  json += ",\"autoplay\":" + String(g_autoplayEnabled ? "true" : "false");
  json += ",\"watchdog\":" + String(g_watchdogEnabled ? "true" : "false");
  json += ",\"bgEffect\":{\"enabled\":" + String(g_bgEffectEnabled ? "true" : "false") +
          ",\"active\":" + String(g_bgEffectActive ? "true" : "false") +
          ",\"path\":\"" + htmlEscape(g_bgEffectPath) + "\"}";
  json += ",\"strobe\":{\"enable\":" + String(g_strobeEnable ? "true" : "false") +
          ",\"deg\":" + String(g_strobeWidthDeg,2) +
          ",\"phase\":" + String(g_strobePhaseDeg,2) + "}";
  json += ",\"rpm\":" + String(computeRpmSnapshot());
  json += ",\"rpmPpr\":" + String((unsigned)g_pulsesPerRev);
  json += ",\"rpmEdge\":" + String((unsigned)g_hallEdgeMode);
  json += ",\"period_us\":" + String(g_lastPeriodUs);
  json += ",\"outmode\":\""; json += (g_outputMode==OUT_PARALLEL?"parallel":"spi"); json += "\"";
  json += ",\"map\":{\"usePerArm\":" + String(g_usePerArmStart ? "true":"false") +
          ",\"start\":" + String(g_startChArm1) +
          ",\"start2\":" + String(g_startChArm[1]) +
          ",\"start3\":" + String(g_startChArm[2]) +
          ",\"start4\":" + String(g_startChArm[3]) + "}";
  json += "}";
  server.send(200,"application/json",json);
}

static void handleDiagTiming() {
  String json = "{";
  json += "\"targetFps\":" + String(g_fps);
  json += ",\"measuredFps\":" + String(g_measuredFps, 2);
  json += ",\"targetPeriod_us\":" + String((unsigned long)g_framePeriodUs);
  json += ",\"spokeDuration_us\":" + String((unsigned long)g_spokeDurationUs);
  json += ",\"lastPeriod_us\":" + String((unsigned long)g_lastPeriodUs);
  json += ",\"frameCounter\":" + String((unsigned long)g_frameCounter);
  json += ",\"sdFailStreak\":" + String(g_sdFailStreak);

  // Calculate current drift
  uint32_t nowUs = micros();
  int32_t drift = (int32_t)(g_lastTickUs + g_framePeriodUs - nowUs);
  json += ",\"drift_us\":" + String((long)drift);

  // Display duty cycle info
  json += ",\"dutyPercent\":" + String((unsigned)g_displayDutyPercent);
  uint32_t holdUs = computeArmHoldDurationUs();
  json += ",\"holdDuration_us\":" + String((unsigned long)holdUs);

  json += "}";

  server.send(200, "application/json", json);
}

static void handleDiagDuty() {
  uint32_t base = g_spokeDurationUs;
  uint32_t hold = computeArmHoldDurationUs();
  uint32_t period = g_lastPeriodUs;

  String json = "{";
  json += "\"dutyPercent\":" + String((unsigned)g_displayDutyPercent);
  json += ",\"spokeDuration_us\":" + String((unsigned long)g_spokeDurationUs);
  json += ",\"lastPeriod_us\":" + String((unsigned long)g_lastPeriodUs);
  json += ",\"computedHold_us\":" + String((unsigned long)hold);
  json += ",\"minHold_us\":" + String((unsigned long)ARM_BLANK_MIN_US);
  json += ",\"fallback_us\":" + String((unsigned long)ARM_BLANK_FALLBACK_US);

  // Show per-arm state
  json += ",\"arms\":[";
  const uint8_t arms = activeArmCount();
  for (uint8_t a = 0; a < arms; ++a) {
    if (a > 0) json += ",";
    json += "{\"lit\":" + String(g_armState[a].lit ? "true" : "false");
    json += ",\"deadline\":" + String((unsigned long)g_armState[a].blankDeadlineUs);
    json += ",\"spoke\":" + String((unsigned)g_armState[a].currentSpoke);
    json += "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

static void handleLogsPage() {
  String page = WebPages::logsPage(DebugLog::logsHtmlPre());
  server.send(200, "text/html; charset=utf-8", page);
}

static void handleLogsText() {
  server.send(200, "text/plain; charset=utf-8", DebugLog::logsText());
}

static void handleLogsClear() {
  DebugLog::clear();
  server.send(200, "text/plain", "OK");
}

static void handleRoot() {
  String options; listFseqInDir("/", options);
  String bgOptions; listBgEffects(bgOptions, g_bgEffectPath);
  String cur = g_currentPath.length() ? g_currentPath : "(none)";
  String curEsc = htmlEscape(cur);
  String bgDisplay = g_bgEffectPath.length() ? bgEffectDisplayName(g_bgEffectPath) : String("(none)");
  String bgEsc = htmlEscape(bgDisplay);
  String apIp = AP_IP.toString();
  wl_status_t st = WiFi.status();
  bool staConfigured = g_staSsid.length() > 0;
  bool staConnected = (st == WL_CONNECTED);
  bool staConnecting = g_staConnecting && !staConnected;
  String staStatus = staConfigured ? (staConnected ? "Connected" : (staConnecting ? "Connecting" : "Not connected")) : "Not configured";
  String staIp = (staConnected) ? WiFi.localIP().toString() : String("-");
  String html = WebPages::rootPage(String(statusClass()), String(statusText()), curEsc, options,
                                   String(AP_SSID), apIp, String("pov.local"),
                                   g_staSsid, staStatus, staIp, g_stationId,
                                  g_startChArm1, g_spokesTotal, g_armCount, g_pixelsPerArm,
                                  MAX_ARMS, MAX_PIXELS_PER_ARM,
                                  g_fps, g_displayDutyPercent, g_brightnessPercent,
                                  (uint8_t)g_sdPreferredBusWidth, g_sdBaseFreqKHz,
                                   g_sdBusWidth, g_sdFreqKHz, g_sdReady,
                                   g_playing, g_paused, g_autoplayEnabled, g_hallDiagEnabled,
                                   g_armTestEnabled, g_watchdogEnabled, g_bgEffectEnabled, g_bgEffectActive, bgEsc,
                                   bgOptions);

  server.send(200, "text/html; charset=utf-8", html);
}

static void applyDisplayDuty(uint8_t pct){
  if (pct > 100) pct = 100;
  g_displayDutyPercent = pct;
  prefs.putUChar("duty", g_displayDutyPercent);
  persistSettingsToSd();

  uint32_t newHold = computeArmHoldDurationUs();
  uint32_t now = micros();
  const uint8_t arms = activeArmCount();

  // Update currently lit arms with new deadline based on when they were painted
  for (uint8_t a = 0; a < arms; ++a) {
    if (g_armState[a].lit && g_armState[a].paintTimestampUs > 0) {
      // Recalculate deadline from original paint time with new hold duration
      g_armState[a].blankDeadlineUs = g_armState[a].paintTimestampUs + newHold;
      if (g_armState[a].blankDeadlineUs == 0) g_armState[a].blankDeadlineUs = 1;

      // If new deadline already passed, blank immediately
      if (microsReached(now, g_armState[a].blankDeadlineUs)) {
        blankArm(a);
      }
    }
  }

  if (g_needShow) {
    lanesCommit();
  }

  DebugLog::printf("[DUTY] %u%% → hold=%lu us (spoke=%lu us)\n",
                   (unsigned)pct, (unsigned long)newHold,
                   (unsigned long)g_spokeDurationUs);
}

static void applyBrightness(uint8_t pct){
  if (pct>100) pct=100;
  g_brightnessPercent=pct; g_brightness=(uint8_t)((255*pct)/100);
  g_output.setBrightness(g_brightness);
  lanesShowAll();
  prefs.putUChar("brightness", g_brightnessPercent);
  persistSettingsToSd();
}

static void handleB(){
  int pct = -1;
  if (server.hasArg("percent")) pct = server.arg("percent").toInt();
  else if (server.hasArg("p"))  pct = server.arg("p").toInt();
  else if (server.hasArg("v"))  pct = server.arg("v").toInt();
  else if (server.hasArg("value")) pct = server.arg("value").toInt();

  if (pct < 0) { server.send(400, "text/plain", "missing"); return; }
  if (pct > 100) pct = 100;
  applyBrightness((uint8_t)pct);
  server.send(200, "application/json", String("{\"brightness\":") + pct + "}");
}

static void handleDuty(){
  int pct = -1;
  bool provided = false;
  if (server.hasArg("percent")) { pct = server.arg("percent").toInt(); provided = true; }
  else if (server.hasArg("p"))  { pct = server.arg("p").toInt(); provided = true; }
  else if (server.hasArg("v"))  { pct = server.arg("v").toInt(); provided = true; }
  else if (server.hasArg("value")) { pct = server.arg("value").toInt(); provided = true; }

  if (!provided) { server.send(400, "text/plain", "missing"); return; }
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  applyDisplayDuty((uint8_t)pct);
  server.send(200, "application/json", String("{\"duty\":") + (unsigned)g_displayDutyPercent + "}");
}

static void handleStart(){
  if (server.hasArg("path")){
    String p = server.arg("path"); if (!p.startsWith("/")) p="/"+p;
    String why;
    if (!openFseq(p, why)){ server.send(500,"text/plain",String("FSEQ open failed: ")+why); return; }
  }
  if (g_armTestEnabled) {
    g_armTestEnabled = false;
    g_armTestCurrentArm = 0;
    g_armTestColorIdx = 0;
    g_armTestCurrentPixel = 0;
    g_armTestNextStepMs = 0;
  }
  g_playing=true;
  g_paused=false;
  g_lastTickUs = micros();  // CHANGE from millis()
  g_bootMs = millis();
  server.send(200,"application/json","{\"playing\":true}");
}
static void handleStop(){
  g_playing=false;
  g_paused=false;
  g_bgEffectActive = false;
  g_bootMs = millis();
  g_bgEffectNextAttemptMs = g_bootMs;
  blackoutAll();
  server.send(200,"application/json","{\"playing\":false}");
}

static bool parseBoolArg(const String& v) {
  String s = v; s.toLowerCase();
  return (s == "1" || s == "true" || s == "yes" || s == "on");
}

static void handlePause(){
  bool toggle = true;
  bool wantPause = !g_paused;
  if (server.hasArg("pause")) {
    wantPause = parseBoolArg(server.arg("pause"));
    toggle = false;
  } else if (server.hasArg("resume")) {
    wantPause = !parseBoolArg(server.arg("resume"));
    toggle = false;
  } else if (server.hasArg("enable")) {
    wantPause = parseBoolArg(server.arg("enable"));
    toggle = false;
  } else if (server.hasArg("disable")) {
    wantPause = !parseBoolArg(server.arg("disable"));
    toggle = false;
  }

  if (!toggle && wantPause && !g_playing) {
    g_paused = false;
    server.send(409, "application/json", "{\"error\":\"not playing\"}");
    return;
  }

  if (toggle && !g_playing) {
    server.send(409, "application/json", "{\"error\":\"not playing\"}");
    return;
  }

  if (!toggle) g_paused = wantPause && g_playing;
  else g_paused = !g_paused && g_playing;

  if (!g_paused) g_lastTickUs = micros();  // CHANGE from millis()

  server.send(200,"application/json",
              String("{\"paused\":") + (g_paused ? "true" : "false") +
              ",\"playing\":" + (g_playing ? "true" : "false") + "}");
}

static void handleHallDiag(){
  if (!server.hasArg("enable")) {
    server.send(400, "application/json", "{\"error\":\"missing enable\"}");
    return;
  }
  String v = server.arg("enable"); v.toLowerCase();
  bool enable = (v == "1" || v == "true" || v == "on" || v == "yes");

  if (enable) {
    if (g_armTestEnabled) {
      g_armTestEnabled = false;
      g_armTestCurrentArm = 0;
      g_armTestColorIdx = 0;
      g_armTestCurrentPixel = 0;
      g_armTestNextStepMs = 0;
    }
    if (!g_hallDiagEnabled) {
      g_hallDiagEnabled = true;
      g_playing = false;
      g_paused = false;
      g_bgEffectActive = false;
      g_bootMs = millis();
      g_bgEffectNextAttemptMs = g_bootMs;
      g_hallDiagActive = false;
      blackoutAll();
    }
  } else {
    if (g_hallDiagEnabled) {
      g_hallDiagEnabled = false;
      g_hallDiagActive = false;
      g_bootMs = millis();
      g_bgEffectNextAttemptMs = g_bootMs;
      blackoutAll();
    }
  }

  server.send(200, "application/json",
              String("{\"hallDiag\":") + (g_hallDiagEnabled ? "true" : "false") +
              ",\"playing\":" + (g_playing ? "true" : "false") + "}");
}

static void handleArmTest(){
  if (!server.hasArg("enable")) {
    server.send(400, "application/json", "{\"error\":\"missing enable\"}");
    return;
  }

  bool enable = parseBoolArg(server.arg("enable"));

  if (enable) {
    if (!g_armTestEnabled) {
      g_armTestEnabled = true;
      g_armTestCurrentArm = 0;
      g_armTestColorIdx = 0;
      g_armTestCurrentPixel = 0;
      g_armTestNextStepMs = 0;
      g_playing = false;
      g_paused = false;
      g_bgEffectActive = false;
      g_bootMs = millis();
      g_bgEffectNextAttemptMs = g_bootMs;
      if (g_hallDiagEnabled || g_hallDiagActive) {
        g_hallDiagEnabled = false;
        g_hallDiagActive = false;
      }
      blackoutAll();
    }
  } else {
    if (g_armTestEnabled) {
      g_armTestEnabled = false;
      g_armTestCurrentArm = 0;
      g_armTestColorIdx = 0;
      g_armTestCurrentPixel = 0;
      g_armTestNextStepMs = 0;
      g_bootMs = millis();
      g_bgEffectNextAttemptMs = g_bootMs;
      blackoutAll();
    }
  }

  server.send(200, "application/json",
              String("{\"armTest\":") + (g_armTestEnabled ? "true" : "false") + "}");
}

static void handleSpeed() {
  if (!server.hasArg("fps")) { server.send(400, "text/plain", "missing fps"); return; }
  int val = server.arg("fps").toInt();
  if (val < 1) val = 1;
  if (val > 120) val = 120;
  g_fps = (uint16_t)val;

  // CRITICAL: Calculate in microseconds, not milliseconds!
  g_framePeriodUs = (uint32_t)(1000000UL / g_fps);

  prefs.putUShort("fps", g_fps);
  persistSettingsToSd();

  g_lastTickUs = micros(); // Use micros, not millis!

  DebugLog::printf("[PLAY] FPS=%u  period=%luus\n", g_fps, (unsigned long)g_framePeriodUs);
  server.send(200, "application/json", String("{\"fps\":") + g_fps + "}");
}

static void handleMapCfg(){
  bool needRebuild = false;

  if (server.hasArg("start"))  {
    uint32_t v = strtoul(server.arg("start").c_str(), nullptr, 10);
    g_startChArm1 = (v < 1) ? 1 : v;
    prefs.putULong("startch", g_startChArm1);
  }
  if (server.hasArg("spokes")) {
    int v = server.arg("spokes").toInt();
    if (v < 1) v = 1;
    g_spokesTotal = (uint16_t)v;
    prefs.putUShort("spokes", g_spokesTotal);
  }
  if (server.hasArg("arms"))   {
    int v = server.arg("arms").toInt();
    uint8_t nv = clampArmCount(v);
    if (nv != g_armCount) {
      g_armCount = nv;
      prefs.putUChar("arms", g_armCount);
      needRebuild = true;
    }
  }
  if (server.hasArg("pixels")) {
    int v = server.arg("pixels").toInt();
    uint16_t np = clampPixelsPerArm(v);
    if (np != g_pixelsPerArm) {
      g_pixelsPerArm = np;
      prefs.putUShort("pixels", g_pixelsPerArm);
      needRebuild = true;
    }
  }
  if (server.hasArg("useperarm")) {
    g_usePerArmStart = parseBoolArg(server.arg("useperarm"));
    prefs.putBool("usepa", g_usePerArmStart);
  }
  if (!g_usePerArmStart) {
    computeDefaultArmStarts(g_startChArm1);
  }

  bool perArmChanged = false;
  if (server.hasArg("start2")) { uint32_t v=strtoul(server.arg("start2").c_str(),nullptr,10); g_startChArm[1]=v; prefs.putULong("start2",v); perArmChanged=true; }
  if (server.hasArg("start3")) { uint32_t v=strtoul(server.arg("start3").c_str(),nullptr,10); g_startChArm[2]=v; prefs.putULong("start3",v); perArmChanged=true; }
  if (server.hasArg("start4")) { uint32_t v=strtoul(server.arg("start4").c_str(),nullptr,10); g_startChArm[3]=v; prefs.putULong("start4",v); perArmChanged=true; }
  if (!g_usePerArmStart && (server.hasArg("start") || perArmChanged)) {
    computeDefaultArmStarts(g_startChArm1);
  }

  if (needRebuild) rebuildStrips();

  persistSettingsToSd();

  server.send(200, "application/json",
    String("{\"start\":") + g_startChArm1 +
    ",\"spokes\":" + g_spokesTotal +
    ",\"arms\":" + (int)g_armCount +
    ",\"pixels\":" + g_pixelsPerArm + "}"
  );
}

static void handleWifiCfg(){
  bool changed = false;
  bool reconnect = false;

  if (server.hasArg("forget")) {
    g_staSsid = ""; g_staPass = "";
    prefs.putString("sta_ssid", g_staSsid);
    prefs.putString("sta_pass", g_staPass);
    changed = true; reconnect = true;
  } else {
    if (server.hasArg("ssid")) {
      String ssid = server.arg("ssid"); ssid.trim();
      g_staSsid = ssid;
      prefs.putString("sta_ssid", g_staSsid);
      changed = true; reconnect = true;
    }
    if (server.hasArg("pass")) {
      g_staPass = server.arg("pass");
      prefs.putString("sta_pass", g_staPass);
      changed = true; reconnect = true;
    }
  }

  if (server.hasArg("station")) {
    String station = server.arg("station"); station.trim();
    if (!station.length()) station = defaultStationId();
    g_stationId = station;
    prefs.putString("station", g_stationId);
    changed = true; reconnect = true;
  }

  if (changed) persistSettingsToSd();

  applyStationHostname();

  if (reconnect) {
    if (g_staSsid.length()) connectWifiStation();
    else { WiFi.disconnect(false, true); g_staConnecting = false; markStationState(false); }
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

static void handleAutoplay(){
  if (!server.hasArg("enable")) {
    server.send(400, "application/json", "{\"error\":\"missing parameters\"}");
    return;
  }
  bool enable = parseBoolArg(server.arg("enable"));
  g_autoplayEnabled = enable;
  prefs.putBool("autoplay", g_autoplayEnabled);
  persistSettingsToSd();
  g_bootMs = millis();

  server.send(200, "application/json", String("{\"autoplay\":") + (g_autoplayEnabled ? "true" : "false") + "}");
}

static void handleWatchdog(){
  if (!server.hasArg("enable")) {
    server.send(400, "application/json", "{\"error\":\"missing enable\"}");
    return;
  }
  bool enable = parseBoolArg(server.arg("enable"));
  g_watchdogEnabled = enable;
  prefs.putBool("watchdog", g_watchdogEnabled);
  persistSettingsToSd();
  applyWatchdogSetting();
  server.send(200, "application/json", String("{\"watchdog\":") + (g_watchdogEnabled ? "true" : "false") + "}");
}

static void handleBgEffect(){
  bool hasEnable = server.hasArg("enable");
  bool hasPath = server.hasArg("path");
  if (!hasEnable && !hasPath) {
    server.send(400, "application/json", "{\"error\":\"missing parameters\"}");
    return;
  }

  bool enable = g_bgEffectEnabled;
  if (hasEnable) enable = parseBoolArg(server.arg("enable"));

  String newPath = g_bgEffectPath;
  if (hasPath) {
    String raw = server.arg("path");
    String sanitized = sanitizeBgEffectPath(raw);
    if (raw.length() && !sanitized.length()) { server.send(400, "application/json", "{\"error\":\"invalid path\"}"); return; }
    newPath = sanitized;
  }

  bool enableChanged = (enable != g_bgEffectEnabled);
  bool pathChanged   = (newPath != g_bgEffectPath);
  bool stateChanged  = enableChanged || pathChanged;

  if (enableChanged) { g_bgEffectEnabled = enable; prefs.putBool("bge_enable", g_bgEffectEnabled); }
  if (pathChanged)   { g_bgEffectPath    = newPath; prefs.putString("bge_path", g_bgEffectPath); }
  if (stateChanged)  persistSettingsToSd();

  g_bootMs = millis();

  if (!g_bgEffectEnabled || !g_bgEffectPath.length()) {
    if (g_bgEffectActive) {
      g_playing = false; g_paused = false; g_bgEffectActive = false;
      g_bootMs = millis();
      for (uint8_t a=0; a<activeArmCount(); ++a) armClear(a);
    }
    g_bgEffectNextAttemptMs = millis();
  } else {
    if (!g_hallDiagEnabled) {
      if (!g_playing || g_bgEffectActive) {
        String why;
        g_paused = false;
        if (openFseq(g_bgEffectPath, why)) { g_bgEffectNextAttemptMs = millis(); }
        else { DebugLog::printf("[BGE] open fail: %s\n", why.c_str()); g_bgEffectNextAttemptMs = millis() + 5000; }
      } else if (stateChanged) {
        g_bgEffectNextAttemptMs = millis();
      }
    }
  }

  String resp = "{\"bgEffect\":{\"enabled\":";
  resp += (g_bgEffectEnabled ? "true" : "false");
  resp += ",\"active\":";
  resp += (g_bgEffectActive ? "true" : "false");
  resp += ",\"path\":\"";
  resp += htmlEscape(g_bgEffectPath);
  resp += "\"}}";
  server.send(200, "application/json", resp);
}

static void handleFseqHeader(){
  String j="{\"ok\":false}";
  if (g_currentPath.length()) {
    j = String("{\"ok\":true,\"path\":\"")+htmlEscape(g_currentPath)+"\",\"frames\":"+g_fh.frameCount+
        ",\"channels\":"+g_fh.channelCount+",\"stepMs\":"+(int)g_fh.stepTimeMs+
        ",\"comp\":"+(int)g_fh.compType+",\"blocks\":"+g_compCount+
        ",\"sparse\":"+(int)g_fh.sparseCnt+",\"cdo\":"+g_fh.chanDataOffset+
        ",\"perFrame\":"+(g_compPerFrame?"true":"false")+"}";
  }
  server.send(200,"application/json",j);
}

static void handleCBlocks(){
  if (!g_compCount){ server.send(200,"application/json","{\"blocks\":0}"); return; }
  uint32_t show = (g_compCount <= 12) ? g_compCount : 12;
  String s = "{\"blocks\":"+String(g_compCount)+",\"items\":[";
  for (uint32_t i=0;i<show;++i){
    if (i) s+=",";
    s+="{\"i\":"+String(i)+",\"u\":"+String(g_cblocks[i].uSize)+",\"c\":"+String(g_cblocks[i].cSize)+"}";
  }
  if (g_compCount>show) s+=",{\"more\":" + String(g_compCount-show) + "}";
  s+="]}";
  server.send(200,"application/json",s);
}

static void handleDiagMap() {
  if (!g_frameValid || !g_frameBuf) { server.send(409,"application/json","{\"error\":\"no frame\"}"); return; }
  uint8_t arm = server.hasArg("arm") ? (uint8_t)constrain(server.arg("arm").toInt()-1,0,(int)activeArmCount()-1) : 0;
  long _pixReq   = server.hasArg("pix")   ? server.arg("pix").toInt()   : 0L;
  long _spokeReq = server.hasArg("spoke") ? server.arg("spoke").toInt() : (long)currentSpokeIndex();

  uint16_t pix   = (uint16_t)std::max<long>(0L, _pixReq);
  uint16_t spoke = (uint16_t)std::max<long>(0L, _spokeReq);

  const uint8_t arms = activeArmCount();
  const uint16_t pixelCount = armPixelCount();
  const uint16_t spokes = spokesCount();
  const uint32_t expectedPerSpoke = (uint32_t)arms * (uint32_t)pixelCount * 3u;
  const bool perSpokeFrame = (g_fh.channelCount == expectedPerSpoke);

  uint32_t chPerSpoke = expectedPerSpoke;
  if (!perSpokeFrame && spokes>0 && (g_fh.channelCount % spokes)==0) chPerSpoke = g_fh.channelCount / spokes;

  uint32_t base = 0;
  if (g_usePerArmStart) base = (g_startChArm[arm] ? g_startChArm[arm]-1 : 0);
  else {
    const uint32_t armBlockStride = (uint32_t)pixelCount*3u;
    base = (g_startChArm1 ? g_startChArm1-1 : 0) + (uint32_t)arm*armBlockStride;
  }
  if (!perSpokeFrame) base += ((uint32_t)(spoke % (spokes?spokes:1))) * chPerSpoke;

  uint32_t ofs = (uint32_t)pix*3u;
  uint32_t absR = base + ofs;

  int64_t idxR = sparseTranslate(absR);
  uint8_t R=0,G=0,B=0;
  if (idxR>=0 && (idxR+2)<(int64_t)g_fh.channelCount) mapChannels(&g_frameBuf[idxR], R,G,B);

  String j = "{";
  j += "\"arm\":" + String((int)arm+1)
     + ",\"pix\":" + String(pix)
     + ",\"spoke\":" + String(spoke)
     + ",\"perSpokeFrame\":" + String(perSpokeFrame?"true":"false")
     + ",\"chPerSpoke\":" + String(chPerSpoke)
     + ",\"absR\":" + String(absR)
     + ",\"idxR\":" + String((long)idxR)
     + ",\"rgb\":[" + String(R) + "," + String(G) + "," + String(B) + "]"
     + "}";
  server.send(200,"application/json",j);
}

static void handleFseqRanges() {
  String s = "{\"sparse\":" + String((int)g_fh.sparseCnt) + ",\"ranges\":[";
  for (uint8_t i=0;i<g_fh.sparseCnt && i<24;i++) {
    if (i) s += ",";
    s += "{\"i\":" + String(i)
       +  ",\"start\":" + String(g_ranges[i].start)
       +  ",\"count\":" + String(g_ranges[i].count)
       +  ",\"accum\":" + String(g_ranges[i].accum) + "}";
  }
  s += "]}";
  server.send(200,"application/json",s);
}
static void handleDiagBlank() {
  uint32_t hold = computeArmHoldDurationUs();

  String json = "{";
  json += "\"dutyPercent\":" + String((unsigned)g_displayDutyPercent);
  json += ",\"holdTime_us\":" + String((unsigned long)hold);
  json += ",\"spokePeriod_us\":" + String((unsigned long)g_spokeDurationUs);
  json += ",\"outputReady\":" + String(g_output.ready() ? "true" : "false");
  json += ",\"frameBytesPerStrip\":" + String((unsigned)SharedClockProtocol::frameBytes(armPixelCount()));

  json += ",\"arms\":[";
  const uint8_t arms = activeArmCount();
  for (uint8_t a = 0; a < arms; ++a) {
    if (a > 0) json += ",";
    json += "{\"lit\":" + String(g_armState[a].lit ? "true" : "false");
    json += ",\"deadline\":" + String((unsigned long)g_armState[a].blankDeadlineUs);
    json += ",\"paintTime\":" + String((unsigned long)g_armState[a].paintTimestampUs);
    uint32_t now = micros();
    int32_t timeLeft = (int32_t)(g_armState[a].blankDeadlineUs - now);
    json += ",\"timeLeft\":" + String((long)timeLeft);
    json += "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}
/* -------------------- SD Recovery Ladder -------------------- */
static bool recoverSd(const char* reason) {
  DebugLog::printf("[SD] Recover: %s  streak=%d  freq=%lu kHz  CD=%d  width=%u\n",
      reason, g_sdFailStreak, (unsigned long)g_sdFreqKHz,
      PIN_SD_CD >= 0 ? (int)digitalRead(PIN_SD_CD) : -1, (unsigned)g_sdBusWidth);

  if (!cardPresent()) {
    DebugLog::println("[SD] Card not present (CD HIGH). Waiting...");
    uint32_t t0 = millis();
    while (!cardPresent() && millis() - t0 < 5000) {
      delay(50);
      server.handleClient();
      feedWatchdog();
    }
    if (!cardPresent()) return false;
  }

  bool ok=false;

  if (g_sdFailStreak == 1) {
    if (g_currentPath.length()) {
      String why;
      ok = openFseq(g_currentPath, why);
      DebugLog::printf("[SD] Reopen file: %s\n", ok?"OK": why.c_str());
      if (ok) { feedWatchdog(); return true; }
    }
  }

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) return false;
  SD_MMC.end();
  g_sdBusWidth = 0;
  g_sdReady = false;
  pinMode(PIN_SD_CLK, OUTPUT);
  digitalWrite(PIN_SD_CLK, LOW);
  delay(5);
  pinMode(PIN_SD_CLK, INPUT);   // ← fixed here
  SD_UNLOCK();
  delay(50);
  feedWatchdog();

  if (g_sdFailStreak >= 2) {
    uint32_t lowered = nextLowerSdFreq(g_sdFreqKHz);
    if (lowered != g_sdFreqKHz) g_sdFreqKHz = lowered;
  }

  ok = mountSdmmc();
  if (ok && g_currentPath.length()) {
    String why;
    ok = openFseq(g_currentPath, why);
    DebugLog::printf("[SD] Reopen after remount: %s\n", ok?"OK": why.c_str());
  }

  if (ok) g_sdFailStreak = 0;
  feedWatchdog();
  return ok;
}

/* -------------------- Strobe & per-arm phase handlers -------------------- */
static void handleStrobe() {
  bool haveEnable = server.hasArg("enable");
  bool haveDeg    = server.hasArg("deg");
  bool havePhase  = server.hasArg("phase");
  if (!haveEnable && !haveDeg && !havePhase) { server.send(400, "application/json", "{\"error\":\"missing parameters\"}"); return; }
  if (haveEnable) { g_strobeEnable = parseBoolArg(server.arg("enable")); prefs.putBool("strb_e", g_strobeEnable); }
  if (haveDeg) {
    g_strobeWidthDeg = server.arg("deg").toFloat();
    if (g_strobeWidthDeg < 0.1f) g_strobeWidthDeg = 0.1f;
    if (g_strobeWidthDeg > 10.0f) g_strobeWidthDeg = 10.0f;
    prefs.putFloat("strb_deg", g_strobeWidthDeg);
  }
  if (havePhase) {
    g_strobePhaseDeg = server.arg("phase").toFloat();
    while (g_strobePhaseDeg < -180.f) g_strobePhaseDeg += 360.f;
    while (g_strobePhaseDeg >  180.f) g_strobePhaseDeg -= 360.f;
    prefs.putFloat("strb_ph", g_strobePhaseDeg);
  }
  persistSettingsToSd();
  server.send(200, "application/json",
              String("{\"strobe\":{\"enable\":") + (g_strobeEnable ? "true":"false") +
              ",\"deg\":" + String(g_strobeWidthDeg,2) +
              ",\"phase\":" + String(g_strobePhaseDeg,2) + "}}");
}

static void handleArmPhase() {
  if (!server.hasArg("arm") || !server.hasArg("deg")) { server.send(400, "application/json", "{\"error\":\"arm & deg required\"}"); return; }
  int arm = server.arg("arm").toInt();
  if (arm < 1 || arm > (int)activeArmCount()) { server.send(400, "application/json", "{\"error\":\"arm out of range\"}"); return; }
  g_armPhaseDeg[arm-1] = server.arg("deg").toFloat();
  server.send(200, "application/json",
              String("{\"arm\":") + arm + ",\"phase\":" + String(g_armPhaseDeg[arm-1],2) + "}");
}

// Retain the old diagnostic URL, reporting measured software output timing.
static void handleDiagSpi() {
  String json = "{\"driver\":\"shared-clock-gpio\",\"ready\":";
  json += g_output.ready() ? "true" : "false";
  json += ",\"clockPin\":" + String(BoardPins::Clock);
  json += ",\"pixelsPerStrip\":" + String(armPixelCount());
  json += ",\"bitsPerStrip\":" + String((unsigned long)(8 * SharedClockProtocol::frameBytes(armPixelCount())));
  json += ",\"lastTransmit_us\":" + String((unsigned long)g_output.lastTransmitUs());
  json += ",\"psramBytes\":" + String((unsigned long)ESP.getPsramSize());
  json += ",\"freePsramBytes\":" + String((unsigned long)ESP.getFreePsram());
  json += ",\"hallPin\":" + String(BoardPins::Hall);
  json += ",\"sdD0Pin\":" + String(BoardPins::SdD0);
  json += ",\"arms\":[";
  for (uint8_t a = 0; a < MAX_ARMS; ++a) {
    if (a) json += ",";
    json += "{\"arm\":" + String(a + 1);
    json += ",\"connector\":" + String(BoardPins::ActiveConnectors[a]);
    json += ",\"dataPin\":" + String(BoardPins::ArmData[a]) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

/* -------------------- RPM config handler -------------------- */
static void handleRpmCfg(){
  bool changed = false;

  if (server.hasArg("ppr")) {
    int p = server.arg("ppr").toInt();
    if (p < 1) p = 1; if (p > 32) p = 32;
    g_pulsesPerRev = (uint8_t)p;
    prefs.putUChar("ppr", g_pulsesPerRev);
    changed = true;
  }

  if (server.hasArg("edge")) {
    String e = server.arg("edge"); e.toLowerCase();
    uint8_t mode = 0; // falling
    if (e == "rising") mode = 1;
    else if (e == "change") mode = 2;
    if (mode != g_hallEdgeMode) {
      g_hallEdgeMode = mode;
      prefs.putUChar("hedge", g_hallEdgeMode);
      attachHallInterrupt();
      changed = true;
    }
  }

  if (changed) persistSettingsToSd();

  String edgeStr = (g_hallEdgeMode==1) ? "rising" : (g_hallEdgeMode==2 ? "change" : "falling");
  String resp = String("{\"ok\":true,\"ppr\":") + g_pulsesPerRev + ",\"edge\":\"" + edgeStr + "\"}";
  server.send(200, "application/json", resp);
}

/* -------------------- Output mode switch -------------------- */
static void handleOutMode() {
  if (!server.hasArg("mode") || server.arg("mode") != "parallel") {
    server.send(400, "application/json", "{\"error\":\"This PCB requires shared-clock parallel output\"}");
    return;
  }
  server.send(200, "application/json", "{\"outmode\":\"parallel\"}");
}

/* -------------------- OTA / Updates page -------------------- */
// (unchanged OTA functions in WebPages/SD_Functions modules)
static void handleReboot() {
  server.send(200, "text/plain", "Rebooting");
  delay(150);
  ESP.restart();
}

/* -------------------- Utility made external for setup() -------------------- */
void ensureBgEffectsDirLocked() {
  if (!SD_MMC.exists("/BGEffects")) {
    SD_MMC.mkdir("/BGEffects");
  }
}

/* -------------------- Server, Setup, Loop -------------------- */
static void startWifiAP(){
  bool haveStation = g_staSsid.length() > 0;
  WiFi.mode(haveStation ? WIFI_AP_STA : WIFI_AP);
  applyStationHostname();
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  applyStationHostname();
  WiFi.setSleep(false);
  if (haveStation) connectWifiStation();
  if (MDNS.begin("pov")) MDNS.addService("http","tcp",80);

  // Control + status
  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/status",  HTTP_GET,  handleStatus);


  // Playback & settings
  server.on("/play",    HTTP_GET,  handlePlayLink);
  server.on("/b",       HTTP_POST, handleB);
  server.on("/duty",    HTTP_POST, handleDuty);
  server.on("/start",   HTTP_GET,  handleStart);
  server.on("/stop",    HTTP_POST, handleStop);
  server.on("/pause",   HTTP_POST, handlePause);
  server.on("/halldiag", HTTP_POST, handleHallDiag);
  server.on("/armtest", HTTP_POST, handleArmTest);
  server.on("/speed",   HTTP_POST, handleSpeed);
  server.on("/mapcfg",  HTTP_POST, handleMapCfg);
  server.on("/wifi",    HTTP_POST, handleWifiCfg);
  server.on("/autoplay",HTTP_POST, handleAutoplay);
  server.on("/watchdog",HTTP_POST, handleWatchdog);
  server.on("/bgeffect",HTTP_POST, handleBgEffect);

  // Connector color diagnostic
  server.on("/lanediag", HTTP_POST, handleLaneDiag);


  // Strobe + per-arm phase
  server.on("/strobe",   HTTP_POST, handleStrobe);
  server.on("/armphase", HTTP_POST, handleArmPhase);

  // RPM configuration
  server.on("/rpm", HTTP_POST, handleRpmCfg);

  // Output mode
  server.on("/outmode", HTTP_POST, handleOutMode);

  // Diagnostics
  server.on("/diag/map",    HTTP_GET,  handleDiagMap);
  server.on("/fseq/ranges", HTTP_GET,  handleFseqRanges);
  server.on("/fseq/header", HTTP_GET,  handleFseqHeader);
  server.on("/fseq/cblocks",HTTP_GET,  handleCBlocks);
  server.on("/sd/reinit",   HTTP_POST, handleSdReinit);
  server.on("/sd/config",   HTTP_POST, handleSdConfig);
  server.on("/diag/blank", HTTP_GET, handleDiagBlank);

  // Files
  server.on("/files",   HTTP_GET,  handleFiles);
  server.on("/dl",      HTTP_GET,  handleDownload);
  server.on("/rm",      HTTP_GET,  handleDelete);
  server.on("/mkdir",   HTTP_GET,  handleMkdir);
  server.on("/ren",     HTTP_GET,  handleRename);

  // Upload FSEQ
  server.on("/upload",  HTTP_POST, handleUploadDone, handleUploadData);

  // Diagnostic Timing
  server.on("/diag/timing", HTTP_GET, handleDiagTiming);
  server.on("/diag/spi", HTTP_GET, handleDiagSpi);
  server.on("/diag/duty", HTTP_GET, handleDiagDuty);

  // Wi-Fi log viewer
  server.on("/logs",       HTTP_GET,  handleLogsPage);
  server.on("/logs.txt",   HTTP_GET,  handleLogsText);
  server.on("/logs/clear", HTTP_POST, handleLogsClear);

  // Updates hub / OTA / FW to SD
  server.on("/updates",    HTTP_GET,  handleUpdatesPage);
  server.on("/ota",        HTTP_GET,  handleOtaPage);
  server.on("/ota",        HTTP_POST, handleOtaFinish, handleOtaData);
  server.on("/fw/upload",  HTTP_POST, handleFwUploadDone, handleFwUploadData);
  server.on("/fw/apply",   HTTP_POST, [](){
    if (!otaAuthOK()) { server.send(401,"text/plain","Unauthorized"); return; }
    checkSdFirmwareUpdate();
    server.send(200,"text/plain","OK");
  });

  // Reboot button endpoint
  server.on("/reboot", HTTP_POST, handleReboot);

  server.onNotFound([](){ server.send(404, "text/plain", String("404 Not Found: ") + server.uri()); });
  server.begin();
  DebugLog::println("[HTTP] WebServer listening on :80");
}

void setup(){
  DebugLog::begin(kEnableSerialDebug);
  DebugLog::println("\n[POV] SK9822 spinner — FSEQ v2 (sparse + zlib per-frame) — four independent outputs on the 16-connector PCB");
  DebugLog::printf("[MAP] labelMode=%d\n", (int)gLabelMode);

  DebugLog::printf("[PSRAM] found=%d total=%lu free=%lu bytes\n", psramFound(),
                   (unsigned long)ESP.getPsramSize(), (unsigned long)ESP.getFreePsram());
  if (!psramFound()) DebugLog::println("[PSRAM] Not detected; select OPI PSRAM for N16R8");
  pinMode(PIN_HALL_SENSOR, INPUT_PULLUP);

  g_sdMutex = xSemaphoreCreateMutex();

  // Restore settings from NVS first
  prefs.begin("display", false);
  // One-time topology migration. Preserve brightness, pixel count, Wi-Fi, etc.
  if (prefs.getUChar("pcb_rev", 0) != 1) {
    prefs.putUChar("arms", BoardPins::InitialArms);
    prefs.putUChar("ppr", PULSES_PER_REV);
    prefs.putUChar("hedge", 0); // Falling edge: one count per magnet pass.
    prefs.putBool("usepa", false); // Old chained-arm channel starts do not apply.
    prefs.putUChar("pcb_rev", 1);
  }
  g_outputMode = OUT_PARALLEL;
  if (!prefs.isKey("outmode") || prefs.getUChar("outmode", OUT_SPI) != OUT_PARALLEL)
    prefs.putUChar("outmode", OUT_PARALLEL);
  PrefPresence present;
  present.sdMode = prefs.isKey("sdmode");
  g_sdPreferredBusWidth = sanitizeSdMode(prefs.getUChar("sdmode", (uint8_t)SD_BUS_AUTO));
  present.sdFreq = prefs.isKey("sdfreq");
  g_sdBaseFreqKHz = sanitizeSdFreq(prefs.getUInt("sdfreq", 8000));
  g_sdFreqKHz = g_sdBaseFreqKHz;
  present.brightness = prefs.isKey("brightness");
  g_brightnessPercent = prefs.getUChar("brightness", 25);
  present.duty = prefs.isKey("duty");
  g_displayDutyPercent = prefs.getUChar("duty", 60);
  if (g_displayDutyPercent > 100) g_displayDutyPercent = 100;
  present.fps = prefs.isKey("fps");
  g_fps = prefs.getUShort("fps", 40);
  present.startCh = prefs.isKey("startch");
  g_startChArm1 = prefs.getULong("startch", 1);
  present.spokes = prefs.isKey("spokes");
  g_spokesTotal = prefs.getUShort("spokes", 40);
  present.arms = prefs.isKey("arms");
  g_armCount = clampArmCount(prefs.getUChar("arms", MAX_ARMS));
  present.pixels = prefs.isKey("pixels");
  g_pixelsPerArm = clampPixelsPerArm(prefs.getUShort("pixels", DEFAULT_PIXELS_PER_ARM));
  present.staSsid = prefs.isKey("sta_ssid");
  g_staSsid = prefs.getString("sta_ssid", "");
  present.staPass = prefs.isKey("sta_pass");
  g_staPass = prefs.getString("sta_pass", "");
  present.station = prefs.isKey("station");
  g_stationId = prefs.getString("station", "");
  present.autoplay = prefs.isKey("autoplay");
  g_autoplayEnabled = prefs.getBool("autoplay", true);
  present.watchdog = prefs.isKey("watchdog");
  g_watchdogEnabled = prefs.getBool("watchdog", false);
  present.bgEffectEnable = prefs.isKey("bge_enable");
  g_bgEffectEnabled = prefs.getBool("bge_enable", false);
  present.bgEffectPath = prefs.isKey("bge_path");
  {
    String storedBg = prefs.getString("bge_path", "");
    g_bgEffectPath = sanitizeBgEffectPath(storedBg);
    if (storedBg.length() && !g_bgEffectPath.length()) prefs.putString("bge_path", g_bgEffectPath);
  }
  // Per-arm mapping prefs
  g_usePerArmStart = prefs.getBool("usepa", false);
  g_startChArm[0]  = g_startChArm1;
  g_startChArm[1]  = prefs.getULong("start2", 0);
  g_startChArm[2]  = prefs.getULong("start3", 0);
  g_startChArm[3]  = prefs.getULong("start4", 0);
  if (!g_usePerArmStart) {
    computeDefaultArmStarts(g_startChArm1);
  } else {
    for (uint8_t a=0; a<activeArmCount(); ++a) {
      if (g_startChArm[a]==0) { computeDefaultArmStarts(g_startChArm1); break; }
    }
  }

  // RPM prefs & ISR
  g_pulsesPerRev = prefs.getUChar("ppr", PULSES_PER_REV);
  if (g_pulsesPerRev < 1) g_pulsesPerRev = 1;
  g_hallEdgeMode = prefs.getUChar("hedge", 0);
  attachHallInterrupt();

  // Prevent an old SD backup from restoring the incompatible SPI topology.
  present.outMode = true;

  // Strobe prefs (NEW): defaults to disabled
  g_strobeEnable   = prefs.getBool("strb_e", false);
  g_strobeWidthDeg = prefs.getFloat("strb_deg", 3.0f);
  g_strobePhaseDeg = prefs.getFloat("strb_ph", 0.0f);

  if (PIN_STROBE_GATE >= 0) { pinMode(PIN_STROBE_GATE, OUTPUT); digitalWrite(PIN_STROBE_GATE, LOW); }

  bool card = cardPresent();
  if (!card) DebugLog::printf("[SD] No card (CD HIGH on GPIO%d); UI still available.\n", PIN_SD_CD);

  if (card) {
    g_sdReady = mountSdmmc();
    if (!g_sdReady) DebugLog::println("[SD] Mount failed; UI still available for diagnostics.");
  }

  if (g_sdReady) checkSdFirmwareUpdate();
  if (g_sdReady) {
    ensureSettingsFromBackup(present);
    if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) { ensureBgEffectsDirLocked(); SD_UNLOCK(); }
  }

  applyWatchdogSetting();

  if (g_brightnessPercent > 100) g_brightnessPercent = 100;
  g_brightness = (uint8_t)((255 * g_brightnessPercent) / 100);
if (!g_fps) g_fps = 40;
  g_framePeriodUs = 1000000UL / g_fps;

  DebugLog::printf("[TIMING] FPS=%u period=%lu us, spokes=%u spoke_period=%lu us\n",
                   g_fps, (unsigned long)g_framePeriodUs,
                   g_spokesTotal,
                   g_spokesTotal ? (unsigned long)(g_framePeriodUs * g_fps / g_spokesTotal) : 0UL);
  if (!g_startChArm1) g_startChArm1 = 1;
  if (!g_spokesTotal) g_spokesTotal = 1;
  g_armCount = clampArmCount(g_armCount);
  g_pixelsPerArm = clampPixelsPerArm(g_pixelsPerArm);
  if (!g_usePerArmStart) computeDefaultArmStarts(g_startChArm1);
  if (!g_stationId.length()) g_stationId = defaultStationId();

  DebugLog::println(F("[Quadrant self-check]"));
  for (int k = 0; k < activeArmCount(); ++k) {
    int s0 = armSpokeIdx0(k, spoke1BasedToIdx0(START_SPOKE_1BASED, SPOKES), SPOKES, activeArmCount());
    DebugLog::printf("Arm %d → spoke %d\n", k+1, s0 + 1);
  }
  DebugLog::printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);
  DebugLog::printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodUs);
  DebugLog::printf("[MAP] startCh(Arm1)=%lu spokes=%u arms=%u pixels/arm=%u\n",
                (unsigned long)g_startChArm1, g_spokesTotal, (unsigned)activeArmCount(),
                (unsigned)g_pixelsPerArm);
  DebugLog::printf("[OUTMODE] %s\n", (g_outputMode==OUT_PARALLEL?"PARALLEL":"SPI"));
  DebugLog::printf("[STROBE] enable=%d width=%.2f phase=%.2f\n", (int)g_strobeEnable, g_strobeWidthDeg, g_strobePhaseDeg);

  startWifiAP();

  if (g_sdReady) {
    if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) {
      uint8_t type = SD_MMC.cardType();
      uint64_t sizeMB = (type==CARD_NONE) ? 0 : (SD_MMC.cardSize() / (1024ULL*1024ULL));
      SD_UNLOCK();
      DebugLog::printf("[SD] Type=%u  Size=%llu MB\n", (unsigned)type, (unsigned long long)sizeMB);
    }
    persistSettingsToSd();
  }

  rebuildStrips();       // Four separate strips on connectors 1, 5, 9, 13
  setDefaultArmPhases();
  blackoutAll();

    // CREATE FRAME MUTEX
  g_frameMutex = xSemaphoreCreateMutex();

  // START DISPLAY TASK ON CORE 1 (high priority)
  xTaskCreatePinnedToCore(
    displayTask,           // Task function
    "DisplayTask",         // Name
    4096,                  // Stack size
    nullptr,               // Parameters
    2,                     // Priority (high)
    &g_displayTaskHandle,  // Task handle
    1                      // Core 1 (Core 0 runs loop())
  );

  DebugLog::println("[DUAL-CORE] Display task pinned to Core 1, frame loading on Core 0");


  g_bootMs   = millis();
  g_playing  = false;
  g_currentPath = "";
  g_rpmUi = 0;
  g_lastRpmUpdateMs = millis();
  g_rpmSampleUs = 0;
  g_rpmLastCount = g_pulseCount;
  g_rpmAccumulatedUs = 0;
  g_rpmAccumulatedPulses = 0;
  DebugLog::println("[STATE] Waiting for selection via web UI (5-min timeout to /test2.fseq)");
}

// CORE 0: Frame loading and web server (runs on core that setup() ran on)
void loop(){
  pollWifiStation();
  server.handleClient();
  updateHallSensor();
  updateArmTest();

  static uint32_t lastRpmPoll = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastRpmPoll >= 250) {
    (void)computeRpmSnapshot();
    lastRpmPoll = nowMs;
  }

  feedWatchdog();

  // Background effect auto-start
  if (g_bgEffectEnabled && g_bgEffectPath.length() && !g_hallDiagEnabled && !g_armTestEnabled && !g_playing) {
    uint32_t now = millis();
    if (now >= g_bgEffectNextAttemptMs) {
      String why;
      g_paused = false;
      if (openFseq(g_bgEffectPath, why)) {
        DebugLog::printf("[BGE] Auto-start %s\n", g_bgEffectPath.c_str());
        g_bgEffectNextAttemptMs = now;
      }
      else {
        DebugLog::printf("[BGE] open fail: %s\n", why.c_str());
        g_bgEffectNextAttemptMs = now + 5000;
      }
    }
  }

  // Autoplay timeout
  if (g_autoplayEnabled && (!g_playing || g_bgEffectActive) && !g_hallDiagEnabled &&
      !g_armTestEnabled && (millis() - g_bootMs > SELECT_TIMEOUT_MS)) {
    String why;
    g_paused = false;
    if (openFseq("/test2.fseq", why)) {
      DebugLog::println("[TIMEOUT] Auto-start /test2.fseq");
    }
    else {
      DebugLog::printf("[TIMEOUT] open fail: %s\n", why.c_str());
      g_bootMs = millis();
    }
  }

  // Early exit if not playing
  if (!g_playing || g_paused) {
    delay(10); // Don't hog CPU
    return;
  }

  // ========== FRAME LOADING (separate from display timing) ==========
  const uint32_t nowUs = micros();

  // Initialize timing on first frame
  if (g_lastTickUs == 0) {
    g_lastTickUs = nowUs;
  }

  // Check if it's time for next frame
  const uint32_t periodUs = g_framePeriodUs ? g_framePeriodUs : 25000;
  const int32_t timeUntilNext = (int32_t)(g_lastTickUs + periodUs - nowUs);

  if (timeUntilNext <= 0) {
    // Time for new frame
    g_lastTickUs = nowUs;

    // Lock frame buffer while loading
    if (g_frameMutex && xSemaphoreTake(g_frameMutex, pdMS_TO_TICKS(10))) {
      if (!loadNextFrame()) {
        ++g_sdFailStreak;
        DebugLog::printf("[PLAY] frame read failed — streak=%d\n", g_sdFailStreak);
        if (!recoverSd("frame read failed")) {
          if (g_sdFailStreak >= 6) {
            DebugLog::println("[SD] Unrecoverable — pausing playback.");
            g_playing = false;
            g_bgEffectActive = false;
            g_bgEffectNextAttemptMs = millis();
            g_sdFailStreak = 0;
            g_frameValid = false;
            blackoutAll();
          }
        }
        xSemaphoreGive(g_frameMutex);
        return;
      }
      xSemaphoreGive(g_frameMutex);
    }

    g_sdFailStreak = 0;
    g_frameCounter++;

    // Log FPS every second
    if (g_lastFpsReportUs == 0) g_lastFpsReportUs = nowUs;
    if (nowUs - g_lastFpsReportUs >= 1000000UL) {
      uint32_t elapsed = nowUs - g_lastFpsReportUs;
      g_measuredFps = (g_frameCounter * 1000000.0f) / (float)elapsed;
      float target = (g_framePeriodUs > 0) ? (1000000.0f / (float)g_framePeriodUs) : 0.0f;

      DebugLog::printf("[FPS] target=%.2f measured=%.2f frames=%lu drift=%ldus\n",
                    target, g_measuredFps, (unsigned long)g_frameCounter, (long)timeUntilNext);

      g_frameCounter = 0;
      g_lastFpsReportUs = nowUs;
    }
  }

  feedWatchdog();
}

// CORE 1: Real-time display update (tight loop, no delays)
void displayTask(void* parameter) {
  g_displayThreadRunning = true;
  DebugLog::println("[DISPLAY] Real-time task started on Core 1");

  while (g_displayThreadRunning) {
    const uint32_t nowUs = micros();

    // Early exit if not playing
    if (!g_playing || g_paused) {
      if (PIN_STROBE_GATE >= 0) digitalWrite(PIN_STROBE_GATE, LOW);
      vTaskDelay(1); // Let the idle task run while waiting for playback.
      continue;
    }

    // Lock frame buffer briefly while reading
    bool haveLock = (g_frameMutex && xSemaphoreTake(g_frameMutex, 0) == pdTRUE);

    if (!haveLock) {
      // Frame is being loaded, skip this iteration
      delayMicroseconds(50);
      continue;
    }

    // ========== DISPLAY UPDATE (critical timing path) ==========
    const uint16_t spokeNow = currentSpokeIndex();

    if (PIN_STROBE_GATE >= 0) {
      bool on = inStrobeWindowForArm(spokeNow, 0);
      digitalWrite(PIN_STROBE_GATE, on ? HIGH : LOW);
    }

    if (g_strobeEnable) {
      processHallSyncEvent(nowUs);
      const uint16_t spokeNow2 = currentSpokeIndex();
      const uint8_t arms = activeArmCount();
      bool anyChange = false;

      for (uint8_t a = 0; a < arms; ++a) {
        const bool in = inStrobeWindowForArm(spokeNow2, a);

        if (in && g_lastPulseSpoke[a] != spokeNow2) {
          paintArmAt(a, spokeNow2, nowUs);
          g_lastPulseSpoke[a] = spokeNow2;
          g_armState[a].lit = true;
          anyChange = true;
        }

        if (!in && g_armState[a].lit) {
          blankArm(a);
          anyChange = true;
        }
      }

      if (anyChange) {
        lanesCommit();
      }
    } else {
      processHallSyncEvent(nowUs);
      advancePredictedSpokes(nowUs);
      processArmBlanking(nowUs); // This is the critical blanking call
    }

    xSemaphoreGive(g_frameMutex);

    // Minimal yield to prevent watchdog timeout
    // This is a tight loop - runs thousands of times per second
  }

  vTaskDelete(nullptr);
}

// Clear just this arm in the shared frame; other arms retain their own colors.
static void blankArm(uint8_t arm) {
  if (arm >= MAX_ARMS) return;
  armClear(arm);
  g_armState[arm].lit = false;
  g_armState[arm].blankDeadlineUs = 0;
}
