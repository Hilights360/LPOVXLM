// POV Spinner — full sketch with RPM counter, OTA, Updates page, and DUAL-SPI lanes (2 posts) 
// Board: ESP32-S3
// Tim Nash (Inventor)
//
// Wiring mode in this build:
//   - Two SPI "posts" (lanes). Each drives TWO arms chained back-to-back.
//   - Lane 0 (reuses prior Arm1 pins): CLK=47, DATA=45 drives Arm1 then Arm2
//       Arm1 = outside-fed (normal direction; index 0 at the outside)
//       Arm2 = center-fed   (reversed direction; index 0 at the center input)
//   - Lane 1 (reuses prior Arm3 pins): CLK=38, DATA=39 drives Arm3 then Arm4
//       Arm3 = outside-fed (normal)
//       Arm4 = center-fed   (reversed)
//
// Notes:
//   * All timing / strobe / Hall logic unchanged.
//   * Per-arm drawing now routes pixels into lane segments with per-arm reverse support.
//   * If you need different two reused ports, change LANE_CLK[] / LANE_DATA[] below.
//   * OUT_SPI remains the default. Parallel mode left available but not used in this wiring.
//
// === Linkage fixes ===
//  - g_brightness is now global (not static) so SD_Functions.cpp can link to it.
//  - feedWatchdog() and openFseq(...) now have external linkage (not static).
//  - ensureBgEffectsDirLocked() is defined here with external linkage so setup() links.

#include "ConfigTypes.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Adafruit_NeoPixel.h>
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

// ---------- NEW: Output mode (SPI vs Parallel-GPIO) ----------
enum OutputMode : uint8_t { OUT_SPI = 0, OUT_PARALLEL = 1 };
uint8_t g_outputMode = OUT_SPI; // persisted in NVS (key: "outmode")

// Parallel-GPIO driver state (kept for compatibility, not used in this wiring)
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
static uint32_t g_clkMask1 = 0;
static uint32_t g_dataMask1[MAX_ARMS] = {0};
static uint32_t g_allDataMask1 = 0;
static bool     g_parallelPinsInit = false;
static volatile uint8_t g_clkPadNops = 0;
static inline void clk_pad_delay() { for (uint8_t i=0;i<g_clkPadNops;i++) __asm__ __volatile__("nop"); }
static inline uint32_t mask1_for_pin(int gpio) { return (gpio >= 32) ? (1u << (gpio - 32)) : 0u; }
static void configureParallelPins();
static void sk9822_tx_parallel_spoke(uint16_t spokeIdx);
static void sk9822_tx_parallel_black();

// ---------- Hall effect + status pixel ----------
static const int      PIN_HALL_SENSOR        = 5;   // A3144 on this pin (LOW when magnet present)
static const int      PIN_STATUS_PIXEL       = 48;

static Adafruit_NeoPixel g_statusPixel(1, PIN_STATUS_PIXEL, NEO_GRB + NEO_KHZ800);
static bool              g_hallPrevActive   = false;
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

// RPM measurement (A3144)
static const uint8_t     PULSES_PER_REV      = 1;   // default; can override at runtime via /rpm
volatile uint32_t        g_lastPeriodUs      = 0;   // last valid pulse period (us)
volatile uint32_t        g_pulseCount        = 0;   // total pulses seen
volatile uint32_t        g_lastPulseUsIsr    = 0;   // last pulse timestamp (us) in ISR
static uint32_t          g_rpmUi             = 0;   // filtered RPM for UI/status
static uint32_t          g_lastRpmUpdateMs   = 0;

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

// ===== PAST 4-ARM PINS (kept for reference; not used directly now) =====
// static const int ARM_CLK[MAX_ARMS]  = { 47, 42, 38, 35 };
// static const int ARM_DATA[MAX_ARMS] = { 45, 41, 39, 36 };

// ===== NEW: TWO-LANE (TWO-POST) SPI =====
static const uint8_t NUM_LANES = 2;
// Reuse two of the original four ports (edit here if you want different ones):
/* This is now setup to use both SPI lanes at full clock speed. 
Arm 1 is outside fed and feeds Arm 2 from the center and it extends 90 degrees from Arm 1 Clockwise as viewed from the Pixels. 
Arm 3 and 4 follow suit with Arm 3 being outside fed and 4 fed from the center. Arm 3 has been moved to the Port for 
Arm 2 to keep from drawing too much current accross the entire PCB*/
static const int LANE_CLK[NUM_LANES]  = { 47, 35 }; // old Arm1 CLK, old Arm3 CLK
static const int LANE_DATA[NUM_LANES] = { 45, 36 }; // old Arm1 DATA, old Arm3 DATA

// Lane DotStar objects (each lane drives two arms chained)
static Adafruit_DotStar* g_lanes[NUM_LANES] = { nullptr, nullptr };

// Virtual "per-arm" routing description into lanes
struct ArmRoute {
  uint8_t  lane;      // 0 or 1
  uint16_t offset;    // start index inside lane
  bool     reverse;   // true if arm is center-fed (data enters at the "end")
};

// Filled in rebuildStrips()
static ArmRoute g_armRoute[MAX_ARMS];

extern uint16_t g_pixelsPerArm;


// Keep legacy pointer array to avoid compile guards in parallel helpers
Adafruit_DotStar* strips[MAX_ARMS] = { nullptr };

// Brightness (0..255 computed from percent)
// was: static uint8_t g_brightness = 63;
uint8_t g_brightness = 63;

// Helpers for per-arm pixel routing into lanes
static inline uint16_t armPixelCount() { return (g_pixelsPerArm ? g_pixelsPerArm : DEFAULT_PIXELS_PER_ARM); }

static inline uint16_t laneIndexForArmPixel(uint8_t arm, uint16_t pixel) {
  const ArmRoute &r = g_armRoute[arm];
  const uint16_t n  = armPixelCount();
  uint16_t local    = r.reverse ? (n - 1 - pixel) : pixel;
  return r.offset + local;
}

static inline void armSetPixel(uint8_t arm, uint16_t pixel, uint8_t R, uint8_t G, uint8_t B) {
  const ArmRoute &r = g_armRoute[arm];
  if (r.lane >= NUM_LANES || !g_lanes[r.lane]) return;
  uint16_t idx = laneIndexForArmPixel(arm, pixel);
  g_lanes[r.lane]->setPixelColor(idx, R, G, B);
}

static inline void armShow(uint8_t arm) {
  const ArmRoute &r = g_armRoute[arm];
  if (r.lane < NUM_LANES && g_lanes[r.lane]) g_lanes[r.lane]->show();
}

static inline void lanesShowAll() {
  for (uint8_t l=0; l<NUM_LANES; ++l) if (g_lanes[l]) g_lanes[l]->show();
}

static inline void armClear(uint8_t arm) {
  const uint16_t n = armPixelCount();
  for (uint16_t i=0;i<n;++i) armSetPixel(arm, i, 0,0,0);
  armShow(arm);
}

static inline void armFillColor(uint8_t arm, uint8_t R, uint8_t G, uint8_t B) {
  const uint16_t n = armPixelCount();
  for (uint16_t i=0; i<n; ++i) armSetPixel(arm, i, R, G, B);
}

static inline void lanesClearAll() {
  for (uint8_t l=0; l<NUM_LANES; ++l) {
    if (!g_lanes[l]) continue;
    uint16_t total = armPixelCount()*2; // two arms per lane
    for (uint16_t i=0;i<total;++i) g_lanes[l]->setPixelColor(i,0,0,0);
    g_lanes[l]->show();
  }
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
      Serial.printf("[WDT] init/reconfig failed: %d\n", (int)err);
      return;
    }
    if (!g_watchdogAttached) {
      err = esp_task_wdt_add(nullptr);
      if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        g_watchdogAttached = true;
        esp_task_wdt_reset();
        Serial.printf("[WDT] Enabled (timeout %us)\n", (unsigned)WATCHDOG_TIMEOUT_SECONDS);
      } else {
        Serial.printf("[WDT] add failed: %d\n", (int)err);
      }
    }
  } else if (g_watchdogAttached) {
    esp_task_wdt_delete(nullptr);
    esp_task_wdt_deinit();
    g_watchdogAttached = false;
    Serial.println("[WDT] Disabled");
  }
}
// was: static inline void feedWatchdog() { ... }
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

//static bool   g_sdReady             = false;
//static uint8_t g_sdBusWidth         = 0;    // 0=not mounted, 1=1-bit, 4=4-bit
String        g_staSsid;
String        g_staPass;
String        g_stationId;
bool          g_staConnecting       = false;
bool          g_staConnected        = false;
uint32_t      g_staConnectStartMs   = 0;

// ---------- Persisted settings (NVS = flash) ----------
Preferences prefs;
uint8_t  g_brightnessPercent = 25;
uint16_t g_fps               = 40;
uint32_t g_framePeriodMs     = 25;
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
// If true, each arm can start at its own absolute channel (R of pixel 0).
// When false, computed defaults are used from g_startChArm1 + stride rules.
bool     g_usePerArmStart = false;            // NVS key: "usepa"
uint32_t g_startChArm[MAX_ARMS] = {0,0,0,0};  // 1-based absolute channel for Arm1..Arm4 (R)
static void computeDefaultArmStarts(uint32_t startArm1); // fwd decl


//enum StrideMode : uint8_t { STRIDE_SPOKE=0, STRIDE_LED=1 };
StrideMode g_strideMode = STRIDE_SPOKE;

// Rotary index (0..spokes-1)
volatile uint16_t g_indexPosition = 0;

// Playback state
volatile bool  g_playing = false, g_paused = false;
String         g_currentPath;
uint32_t       g_frameIndex = 0, g_lastTickMs = 0;
uint32_t       g_bootMs = 0;
const uint32_t SELECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;

// Arm runtime (timers/blanking)
struct ArmRuntimeState {
  uint16_t baseSpoke = 0;
  uint16_t currentSpoke = 0;
  uint32_t blankDeadlineUs = 0;
  bool     lit = false;
};
static ArmRuntimeState g_armState[MAX_ARMS];
static uint32_t        g_spokeDurationUs     = 0;
static uint32_t        g_nextSpokeDeadlineUs = 0;
static uint16_t        g_spokeStep           = 0;
static bool            g_frameValid          = false;

static inline bool microsReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

static void resetArmRuntimeStates();
static void blankArm(uint8_t arm);
static void paintArmAt(uint8_t arm, uint16_t spokeIdx, uint32_t nowUs);
static void processHallSyncEvent(uint32_t nowUs);
static void advancePredictedSpokes(uint32_t nowUs);
static bool loadNextFrame();

/* -------------------- Strobe gating / angular timing -------------------- */
static const int PIN_STROBE_GATE = -1; // -1 to disable gate pin

static volatile bool  g_strobeEnable   = true;
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
  const bool prevHall   = g_hallPrevActive;

  if (g_hallDiagEnabled) {
    if (hallActive && !g_hallDiagActive) {
      g_hallDiagActive = true;
      const uint8_t arms = (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount);
      // Fill all arms red, show once per lane
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

  if (hallActive != prevHall) {
    g_statusPixel.setPixelColor(0, hallActive ? g_statusPixel.Color(255, 255, 255) : 0);
    g_statusPixel.show();
  }

  g_hallPrevActive = hallActive;
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

// was: static bool openFseq(const String& path, String& why)
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
    g_frameBuf = (uint8_t*)malloc(g_fh.channelCount);
    if (!g_frameBuf){ why="oom frame"; break; }

    g_currentPath = path;
    g_frameIndex = 0;
    g_bgEffectActive = g_bgEffectEnabled && isBgEffectPath(g_currentPath);

    if (g_fh.stepTimeMs > 0) {
      g_framePeriodMs = (uint32_t)g_fh.stepTimeMs;
      if (g_framePeriodMs == 0) g_framePeriodMs = 1;
      uint32_t fpsCalc = (1000UL + (uint32_t)g_fh.stepTimeMs / 2UL) / (uint32_t)g_fh.stepTimeMs;
      if (fpsCalc == 0) fpsCalc = 1;
      g_fps = (uint16_t)clampI32((int32_t)fpsCalc, 1, 1000);
    }

    ok = true;
  } while(0);
  SD_UNLOCK();

  if (ok) {
    resetArmRuntimeStates();
    g_frameValid = false;
    if (!loadNextFrame()) { why = "frame load"; ok = false; }
    else {
      g_lastTickMs = millis();
      g_playing = true;
      g_paused = false;
      Serial.printf("[FSEQ] %s frames=%lu chans=%lu step=%ums comp=%u blocks=%u sparse=%u CDO=0x%04x\n",
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
          uint8_t* nb = (uint8_t*)realloc(s_ctmp, clen);
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

/* -------------------- Color & stride mapping -------------------- */
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


// Compute per-arm starting channels from g_startChArm1 according to stride.
// Results are 1-based absolute channel numbers (R of pixel 0 for that arm).
static void computeDefaultArmStarts(uint32_t startArm1) {
  if (startArm1 < 1) startArm1 = 1;

  const uint8_t arms = activeArmCount();
  const uint16_t pix = armPixelCount();

  // Clear all first
  for (uint8_t a = 0; a < MAX_ARMS; ++a) g_startChArm[a] = 0;

  if (g_strideMode == STRIDE_SPOKE) {
    // [Arm1 block][Arm2 block]...[ArmN block], each block = pix * 3 channels
    for (uint8_t a = 0; a < arms; ++a) {
      g_startChArm[a] = startArm1 + (uint32_t)a * (uint32_t)pix * 3u;
    }
  } else {
    // STRIDE_LED: LED-interleaved per pixel; Arm K’s pixel0 R is + (K * 3)
    for (uint8_t a = 0; a < arms; ++a) {
      g_startChArm[a] = startArm1 + (uint32_t)a * 3u;
    }
  }
}

/* -------------------- Rebuild TWO-LANE strips and arm routes -------------------- */
static void rebuildStrips(){
  // Dispose old lanes
  for (uint8_t l=0;l<NUM_LANES;++l) {
    if (g_lanes[l]) { delete g_lanes[l]; g_lanes[l] = nullptr; }
  }
  // Build new lanes (each drives 2 * pixelsPerArm)
  const uint16_t nPerArm = armPixelCount();
  const uint16_t nPerLane = nPerArm * 2;
  for (uint8_t l=0;l<NUM_LANES;++l) {
    g_lanes[l] = new Adafruit_DotStar(nPerLane, LANE_DATA[l], LANE_CLK[l], DOTSTAR_BGR);
    g_lanes[l]->begin();
    g_lanes[l]->setBrightness(g_brightness);
    g_lanes[l]->clear();
    g_lanes[l]->show();
  }

  // Route table: Arm1+Arm2 on Lane0 ; Arm3+Arm4 on Lane1
  // Arm1 outside-fed (normal), Arm2 center-fed (reversed)
  // Arm3 outside-fed (normal), Arm4 center-fed (reversed)
  g_armRoute[0] = { 0, 0,           false };         // Arm1 → lane 0, offset 0, normal
  g_armRoute[1] = { 0, nPerArm,     true  };         // Arm2 → lane 0, offset N, reversed
  g_armRoute[2] = { 1, 0,           false };         // Arm3 → lane 1, offset 0, normal
  g_armRoute[3] = { 1, nPerArm,     true  };         // Arm4 → lane 1, offset N, reversed

  // Clear legacy arm state
  for (uint8_t a=0;a<MAX_ARMS;++a) strips[a] = nullptr;

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
//Diagnostic to see which SPI Lane is being used for what arm
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


/* -------------------- Parallel helpers (unchanged behavior) -------------------- */
static void configureParallelPins() {
  if (g_parallelPinsInit) return;
  g_clkMask1 = 0;
  g_allDataMask1 = 0;
  // No dedicated ARM_CLK/DATA used now; keep masks zeroed (safe no-op if parallel selected)
  g_parallelPinsInit = true;
}

static IRAM_ATTR void sk9822_tx_parallel_spoke(uint16_t spokeIdx) {
  // Fallback-safe using g_pixelsPerArm if no strip object
  const uint16_t pixelCount = armPixelCount();
  if (!spokesCount() || !pixelCount) return;

  // Minimal parallel stub clocks (pins/masks are zero in this SPI build)
  noInterrupts();
  for (int k = 0; k < 32; ++k) { GPIO.out1_w1tc.val = g_allDataMask1; GPIO.out1_w1ts.val = g_clkMask1; clk_pad_delay(); GPIO.out1_w1tc.val = g_clkMask1; }
  for (uint16_t i = 0; i < pixelCount; ++i) {
    for (int bit = 0; bit < 32; ++bit) { GPIO.out1_w1tc.val = g_allDataMask1; GPIO.out1_w1ts.val = g_clkMask1; clk_pad_delay(); GPIO.out1_w1tc.val = g_clkMask1; }
  }
  for (int k = 0; k < 32; ++k) { GPIO.out1_w1tc.val = g_allDataMask1; GPIO.out1_w1ts.val = g_clkMask1; clk_pad_delay(); GPIO.out1_w1tc.val = g_clkMask1; }
  interrupts();
}

static IRAM_ATTR void sk9822_tx_parallel_black() {
  noInterrupts();
  for (int k = 0; k < 32; ++k) { GPIO.out1_w1tc.val = g_allDataMask1; GPIO.out1_w1ts.val = g_clkMask1; clk_pad_delay(); GPIO.out1_w1tc.val = g_clkMask1; }
  const uint16_t pixelCount  = armPixelCount();
  const uint32_t bits = 32u * pixelCount + 32u;
  for (uint32_t b = 0; b < bits; ++b) { GPIO.out1_w1tc.val = g_allDataMask1; GPIO.out1_w1ts.val = g_clkMask1; clk_pad_delay(); GPIO.out1_w1tc.val = g_clkMask1; }
  interrupts();
}

/* -------------------- Draw / blank on SPI lanes -------------------- */
static void blackoutAll(){
  for (uint8_t a=0; a<MAX_ARMS; ++a) blankArm(a);
  resetArmRuntimeStates();
}

static void paintArmAt(uint8_t arm, uint16_t spokeIdx, uint32_t nowUs){
  if (arm >= MAX_ARMS) return;

  // === Parallel fallback (unchanged) ===
  if (g_outputMode == OUT_PARALLEL) {
    if (arm == 0) {
      configureParallelPins();
      sk9822_tx_parallel_spoke(spokeIdx);
      const uint8_t arms = activeArmCount();
      const uint16_t sct = spokesCount();
      for (uint8_t a = 0; a < arms; ++a) {
        g_armState[a].currentSpoke = sct ? (spokeIdx % sct) : 0;
        g_armState[a].lit = true;
        g_armState[a].blankDeadlineUs = 0;
      }
    }
    return;
  }

  // === SPI (two-lane) path — FIXED indexing ===
  const uint16_t spokes = spokesCount();
  const uint8_t  arms   = activeArmCount();
  const uint16_t pixelCount = armPixelCount();

  if (!g_frameValid || !g_frameBuf || g_fh.channelCount == 0 || arms == 0 || pixelCount == 0) {
    blankArm(arm);
    return;
  }

  // Many spinner FSEQ exports store ONE spoke per frame:
  //   channelCount == arms * pixelsPerArm * 3
  const uint32_t expectedPerSpoke = (uint32_t)arms * (uint32_t)pixelCount * 3u;
  const bool perSpokeFrame = (g_fh.channelCount == expectedPerSpoke);

  // Channels that belong to ONE spoke inside this frame
  uint32_t chPerSpoke = expectedPerSpoke;
  if (!perSpokeFrame) {
    if (spokes > 0 && (g_fh.channelCount % spokes) == 0) {
      chPerSpoke = g_fh.channelCount / spokes;
    }
  }

  // Base (R) channel for THIS arm, pixel 0 (1-based → 0-based)
  uint32_t baseChAbsR = 0;
  if (g_usePerArmStart) {
    baseChAbsR = (g_startChArm[arm] ? g_startChArm[arm]-1 : 0);
  } else {
    const uint32_t base = (g_startChArm1 ? g_startChArm1-1 : 0);
    const uint32_t armBlockStride2 = (uint32_t)pixelCount * 3u;
    baseChAbsR = (g_strideMode == STRIDE_SPOKE)
      ? (base + (uint32_t)arm * armBlockStride2)
      : (base + (uint32_t)arm * 3u);
  }

  // If a frame carries multiple spokes, add the spoke slice offset
  if (!perSpokeFrame) {
    const uint16_t s = (spokes ? spokes : 1);
    baseChAbsR += ((uint32_t)(spokeIdx % s)) * chPerSpoke;
  }

  // Within a spoke, choose addressing pattern:
  //  - STRIDE_SPOKE: [Arm1 block][Arm2 block]...[ArmN block] (each block = pixelsPerArm*3)
  //  - STRIDE_LED:   LED-interleaved per pixel: [(A1 RGB)(A2 RGB)...(AN RGB)] then next pixel
  const uint32_t armBlockStride   = (uint32_t)pixelCount * 3u; // bytes per arm within a spoke
  const uint32_t ledInterleaved3A = (uint32_t)arms * 3u;       // bytes to next LED group when interleaved

  if (spokes) spokeIdx %= spokes;
  g_armState[arm].currentSpoke = spokeIdx;

  for (uint16_t i = 0; i < pixelCount; ++i) {
    uint32_t ofs;
    if (g_strideMode == STRIDE_SPOKE) {
      ofs = (g_usePerArmStart ? 0u : (uint32_t)arm * armBlockStride) + (uint32_t)i * 3u;
    } else { // STRIDE_LED
      const uint32_t ledInterleaved3A2 = (uint32_t)arms * 3u;
      ofs = (uint32_t)i * ledInterleaved3A2 + (g_usePerArmStart ? 0u : (uint32_t)arm * 3u);
    }

    const uint32_t absR = baseChAbsR + ofs;
    const int64_t  idxR = sparseTranslate(absR);

    uint8_t R=0,G=0,B=0;
    if (idxR >= 0 && (idxR + 2) < (int64_t)g_fh.channelCount) {
      mapChannels(&g_frameBuf[idxR], R, G, B);
    }

    if (g_brightness < 255) {
      R = (uint8_t)((uint16_t)R * g_brightness / 255);
      G = (uint8_t)((uint16_t)G * g_brightness / 255);
      B = (uint8_t)((uint16_t)B * g_brightness / 255);
    }
    armSetPixel(arm, i, R, G, B);
  }

  armShow(arm);
  g_armState[arm].lit = true;
  g_armState[arm].blankDeadlineUs = 0;
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

  uint16_t hallBase = (uint16_t)startIdx0 % spokes;
  g_indexPosition = hallBase;

  for (uint8_t a = 0; a < arms; ++a){
    uint32_t offset = ((uint32_t)a * (uint32_t)spokes) / (uint32_t)arms;
    uint32_t baseCalc = ((uint32_t)hallBase + (offset % (uint32_t)spokes)) % (uint32_t)spokes;
    uint16_t base = (uint16_t)baseCalc;
    g_armState[a].baseSpoke = base;
    g_lastPulseSpoke[a] = 0xFFFF;
    g_armState[a].lit = false;

    if (!g_strobeEnable) {
      paintArmAt(a, base, nowUs);
    }
  }
  for (uint8_t a = arms; a < MAX_ARMS; ++a){
    g_armState[a].baseSpoke = 0;
    g_lastPulseSpoke[a] = 0xFFFF;
    g_armState[a].lit = false;
  }

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
  while (microsReached(nowUs, g_nextSpokeDeadlineUs)) {
    g_spokeStep = (g_spokeStep + 1) % spokes;
    uint32_t nextIdx = ((uint32_t)g_indexPosition + 1u) % (uint32_t)spokes;
    g_indexPosition = (uint16_t)nextIdx;
    for (uint8_t a=0; a<arms; ++a){
      uint16_t base = g_armState[a].baseSpoke % spokes;
      uint16_t spoke = (base + g_spokeStep) % spokes;
      paintArmAt(a, spoke, nowUs);
    }
    g_nextSpokeDeadlineUs += g_spokeDurationUs;
  }
}

/* -------------------- Web: Files page + ops -------------------- */
// (unchanged file handlers – omitted comments to keep size down)
/* -------------------- Web: Control page & API -------------------- */
static inline const char* statusText() { if (g_paused && g_playing) return "Paused"; if (g_playing) return "Playing"; return "Stopped"; }
static inline const char* statusClass(){ if (g_paused && g_playing) return "badge pause"; if (g_playing) return "badge play"; return "badge stop"; }

static uint32_t computeRpmSnapshot() {
  uint32_t nowUs = micros();
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

      if (g_pulsesPerRev > 0) {
        uint64_t denom = (uint64_t)g_rpmAccumulatedUs * (uint64_t)g_pulsesPerRev;
        uint64_t num   = (uint64_t)g_rpmAccumulatedPulses * 60000000ULL;
        uint32_t inst  = (denom ? (uint32_t)((num + denom / 2) / denom) : 0);
        if (inst > 0) {
          g_rpmUi = (g_rpmUi * 3 + inst) / 4;
          g_lastRpmUpdateMs = millis();
        }
      }

      g_rpmAccumulatedUs = 0;
      g_rpmAccumulatedPulses = 0;
    } else {
      if (g_rpmAccumulatedUs > 6000000ULL) g_rpmAccumulatedUs = 6000000ULL;
    }
  }

  if (millis() - g_lastRpmUpdateMs > 2000) {
    g_rpmUi = 0;
    g_rpmAccumulatedUs = 0;
    g_rpmAccumulatedPulses = 0;
    g_rpmSampleUs = nowUs;
    g_rpmLastCount = g_pulseCount;
  }
  return g_rpmUi;
}

static void handleStatus(){
  String json = String("{\"playing\":")+(g_playing?"true":"false")
    +",\"paused\":"+(g_paused?"true":"false")
    +",\"path\":\""+htmlEscape(g_currentPath)+"\""
    +",\"frame\":"+String(g_frameIndex)
    +",\"fps\":"+String(g_fps)
    +",\"startChArm1\":"+String(g_startChArm1)
    +",\"spokes\":"+String(g_spokesTotal)
    +",\"arms\":"+String(g_armCount)
    +",\"pixels\":"+String(g_pixelsPerArm)
    +",\"index\":"+String(g_indexPosition)
    +",\"stride\":\""+String(g_strideMode==STRIDE_SPOKE?"spoke":"led")+"\"";
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
                                   g_strideMode == STRIDE_SPOKE, g_fps, g_brightnessPercent,
                                   (uint8_t)g_sdPreferredBusWidth, g_sdBaseFreqKHz,
                                   g_sdBusWidth, g_sdFreqKHz, g_sdReady,
                                   g_playing, g_paused, g_autoplayEnabled, g_hallDiagEnabled,
                                   g_armTestEnabled, g_watchdogEnabled, g_bgEffectEnabled, g_bgEffectActive, bgEsc,
                                   bgOptions);

  server.send(200, "text/html; charset=utf-8", html);
}

static void applyBrightness(uint8_t pct){
  if (pct>100) pct=100;
  g_brightnessPercent=pct; g_brightness=(uint8_t)((255*pct)/100);
  for (uint8_t l=0; l<NUM_LANES; ++l) if (g_lanes[l]) { g_lanes[l]->setBrightness(g_brightness); g_lanes[l]->show(); }
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
  g_playing=true; g_paused=false; g_lastTickMs=millis();
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

  if (!g_paused) g_lastTickMs = millis();

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
  g_framePeriodMs = (uint32_t) (1000UL / g_fps);
  prefs.putUShort("fps", g_fps);
  persistSettingsToSd();
  g_lastTickMs = millis();
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
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
  if (!g_usePerArmStart && (server.hasArg("start") || server.hasArg("stride") || perArmChanged)) {
    computeDefaultArmStarts(g_startChArm1);
  }

  if (needRebuild) rebuildStrips();

  persistSettingsToSd();

  server.send(200, "application/json",
    String("{\"start\":") + g_startChArm1 +
    ",\"spokes\":" + g_spokesTotal +
    ",\"arms\":" + (int)g_armCount +
    ",\"pixels\":" + g_pixelsPerArm +
    ",\"stride\":\"" + (g_strideMode==STRIDE_SPOKE ? "spoke" : "led") + "\"}"
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
        else { Serial.printf("[BGE] open fail: %s\n", why.c_str()); g_bgEffectNextAttemptMs = millis() + 5000; }
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
    base = (g_startChArm1 ? g_startChArm1-1 : 0) + (g_strideMode==STRIDE_SPOKE ? (uint32_t)arm*armBlockStride : (uint32_t)arm*3u);
  }
  if (!perSpokeFrame) base += ((uint32_t)(spoke % (spokes?spokes:1))) * chPerSpoke;

  uint32_t ofs = (g_strideMode==STRIDE_SPOKE) ? (uint32_t)pix*3u : (uint32_t)pix*(uint32_t)arms*3u;
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

/* -------------------- SD Recovery Ladder -------------------- */
static bool recoverSd(const char* reason) {
  Serial.printf("[SD] Recover: %s  streak=%d  freq=%lu kHz  CD=%d  width=%u\n",
      reason, g_sdFailStreak, (unsigned long)g_sdFreqKHz,
      (int)digitalRead(PIN_SD_CD), (unsigned)g_sdBusWidth);

  if (!cardPresent()) {
    Serial.println("[SD] Card not present (CD HIGH). Waiting...");
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
      Serial.printf("[SD] Reopen file: %s\n", ok?"OK": why.c_str());
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
    Serial.printf("[SD] Reopen after remount: %s\n", ok?"OK": why.c_str());
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
  if (haveEnable) { g_strobeEnable = parseBoolArg(server.arg("enable")); }
  if (haveDeg) {
    g_strobeWidthDeg = server.arg("deg").toFloat();
    if (g_strobeWidthDeg < 0.1f) g_strobeWidthDeg = 0.1f;
    if (g_strobeWidthDeg > 10.0f) g_strobeWidthDeg = 10.0f;
  }
  if (havePhase) {
    g_strobePhaseDeg = server.arg("phase").toFloat();
    while (g_strobePhaseDeg < -180.f) g_strobePhaseDeg += 360.f;
    while (g_strobePhaseDeg >  180.f) g_strobePhaseDeg -= 360.f;
  }
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
static void setOutputMode(uint8_t mode) {
  mode = (mode == OUT_PARALLEL) ? OUT_PARALLEL : OUT_SPI;
  if (mode == g_outputMode) return;
  g_outputMode = mode;
  prefs.putUChar("outmode", g_outputMode);
  persistSettingsToSd();
  if (g_outputMode == OUT_PARALLEL) configureParallelPins();
  blackoutAll();
}
static void handleOutMode() {
  if (!server.hasArg("mode")) { server.send(400,"application/json","{\"error\":\"missing mode\"}"); return; }
  String m = server.arg("mode"); m.toLowerCase();
  if (m != "spi" && m != "parallel") { server.send(400,"application/json","{\"error\":\"mode must be spi|parallel\"}"); return; }
  setOutputMode(m == "parallel" ? OUT_PARALLEL : OUT_SPI);
  server.send(200,"application/json", String("{\"outmode\":\"") + (g_outputMode==OUT_PARALLEL?"parallel":"spi") + "\"}");
}

/* -------------------- OTA / Updates page -------------------- */
// (unchanged OTA functions)
static void handleReboot() {
  server.send(200, "text/plain", "Rebooting");
  delay(150);
  ESP.restart();
}

/* -------------------- Utility made external for setup() -------------------- */
// Ensure /BGEffects exists. Caller must hold SD lock if needed.
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
  
//SPI Lane Diag
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

  // Files
  server.on("/files",   HTTP_GET,  handleFiles);
  server.on("/dl",      HTTP_GET,  handleDownload);
  server.on("/rm",      HTTP_GET,  handleDelete);
  server.on("/mkdir",   HTTP_GET,  handleMkdir);
  server.on("/ren",     HTTP_GET,  handleRename);

  // Upload FSEQ
  server.on("/upload",  HTTP_POST, handleUploadDone, handleUploadData);

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
  Serial.println("[HTTP] WebServer listening on :80");
}

void setup(){
  Serial.begin(115200);
  delay(800);
  Serial.println("\n[POV] SK9822 spinner — FSEQ v2 (sparse + zlib per-frame) — DUAL-SPI lanes build");
  Serial.printf("[MAP] labelMode=%d\n", (int)gLabelMode);

  pinMode(PIN_HALL_SENSOR, INPUT_PULLUP);

  g_statusPixel.begin();
  g_statusPixel.setBrightness(64);
  g_statusPixel.clear();
  g_statusPixel.show();

  g_sdMutex = xSemaphoreCreateMutex();

  // Restore settings from NVS first
  prefs.begin("display", false);
  PrefPresence present;
  present.sdMode = prefs.isKey("sdmode");
  g_sdPreferredBusWidth = sanitizeSdMode(prefs.getUChar("sdmode", (uint8_t)SD_BUS_AUTO));
  present.sdFreq = prefs.isKey("sdfreq");
  g_sdBaseFreqKHz = sanitizeSdFreq(prefs.getUInt("sdfreq", 8000));
  g_sdFreqKHz = g_sdBaseFreqKHz;
  present.brightness = prefs.isKey("brightness");
  g_brightnessPercent = prefs.getUChar("brightness", 25);
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
  present.stride = prefs.isKey("stride");
  g_strideMode = (StrideMode)prefs.getUChar("stride", (uint8_t)STRIDE_SPOKE);
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

  // Output mode pref (default SPI)
  g_outputMode = prefs.getUChar("outmode", (uint8_t)OUT_SPI);
  if (g_outputMode != OUT_SPI && g_outputMode != OUT_PARALLEL) g_outputMode = OUT_SPI;
  if (g_outputMode == OUT_PARALLEL) configureParallelPins();

  if (PIN_STROBE_GATE >= 0) { pinMode(PIN_STROBE_GATE, OUTPUT); digitalWrite(PIN_STROBE_GATE, LOW); }

  bool card = cardPresent();
  if (!card) Serial.printf("[SD] No card (CD HIGH on GPIO%d); UI still available.\n", PIN_SD_CD);

  if (card) {
    g_sdReady = mountSdmmc();
    if (!g_sdReady) Serial.println("[SD] Mount failed; UI still available for diagnostics.");
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
  g_framePeriodMs = 1000UL / g_fps;
  if (!g_startChArm1) g_startChArm1 = 1;
  if (!g_spokesTotal) g_spokesTotal = 1;
  g_armCount = clampArmCount(g_armCount);
  g_pixelsPerArm = clampPixelsPerArm(g_pixelsPerArm);
  if (g_strideMode != STRIDE_LED && g_strideMode != STRIDE_SPOKE) g_strideMode = STRIDE_SPOKE;
  if (!g_stationId.length()) g_stationId = defaultStationId();

  Serial.println(F("[Quadrant self-check]"));
  for (int k = 0; k < activeArmCount(); ++k) {
    int s0 = armSpokeIdx0(k, spoke1BasedToIdx0(START_SPOKE_1BASED, SPOKES), SPOKES, activeArmCount());
    Serial.printf("Arm %d → spoke %d\n", k+1, s0 + 1);
  }
  Serial.printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
  Serial.printf("[MAP] startCh(Arm1)=%lu spokes=%u arms=%u pixels/arm=%u stride=%s\n",
                (unsigned long)g_startChArm1, g_spokesTotal, (unsigned)activeArmCount(),
                (unsigned)g_pixelsPerArm, (g_strideMode==STRIDE_SPOKE?"SPOKE":"LED"));
  Serial.printf("[OUTMODE] %s\n", (g_outputMode==OUT_PARALLEL?"PARALLEL":"SPI"));

  startWifiAP();

  if (g_sdReady) {
    if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) {
      uint8_t type = SD_MMC.cardType();
      uint64_t sizeMB = (type==CARD_NONE) ? 0 : (SD_MMC.cardSize() / (1024ULL*1024ULL));
      SD_UNLOCK();
      Serial.printf("[SD] Type=%u  Size=%llu MB\n", (unsigned)type, (unsigned long long)sizeMB);
    }
    persistSettingsToSd();
  }

  rebuildStrips();       // Build two SPI lanes + routes
  setDefaultArmPhases();
  blackoutAll();

  g_bootMs   = millis();
  g_playing  = false;
  g_currentPath = "";
  g_rpmUi = 0;
  g_lastRpmUpdateMs = millis();
  g_rpmSampleUs = 0;
  g_rpmLastCount = g_pulseCount;
  g_rpmAccumulatedUs = 0;
  g_rpmAccumulatedPulses = 0;
  Serial.println("[STATE] Waiting for selection via web UI (5-min timeout to /test2.fseq)");
}

void loop(){
  pollWifiStation();
  server.handleClient();
  updateHallSensor();
  updateArmTest();

  static uint32_t lastRpmPoll = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastRpmPoll >= 250) { (void)computeRpmSnapshot(); lastRpmPoll = nowMs; }

  feedWatchdog();

  if (g_bgEffectEnabled && g_bgEffectPath.length() && !g_hallDiagEnabled && !g_armTestEnabled && !g_playing) {
    uint32_t now = millis();
    if (now >= g_bgEffectNextAttemptMs) {
      String why;
      g_paused = false;
      if (openFseq(g_bgEffectPath, why)) { Serial.printf("[BGE] Auto-start %s\n", g_bgEffectPath.c_str()); g_bgEffectNextAttemptMs = now; }
      else { Serial.printf("[BGE] open fail: %s\n", why.c_str()); g_bgEffectNextAttemptMs = now + 5000; }
    }
  }

  if (g_autoplayEnabled && (!g_playing || g_bgEffectActive) && !g_hallDiagEnabled && !g_armTestEnabled && (millis() - g_bootMs > SELECT_TIMEOUT_MS)) {
    String why;
    g_paused = false;
    if (openFseq("/test2.fseq", why)) { Serial.println("[TIMEOUT] Auto-start /test2.fseq"); }
    else { Serial.printf("[TIMEOUT] open fail: %s\n", why.c_str()); g_bootMs = millis(); }
  }

  if (!g_playing || g_paused) {
    if (PIN_STROBE_GATE >= 0) digitalWrite(PIN_STROBE_GATE, LOW);
    delay(1);
    feedWatchdog();
    return;
  }

  const uint16_t spokeNow = currentSpokeIndex();

  if (PIN_STROBE_GATE >= 0) {
    bool on = inStrobeWindowForArm(spokeNow, 0);
    digitalWrite(PIN_STROBE_GATE, on ? HIGH : LOW);
  }

  const uint32_t now = millis();
  if (now - g_lastTickMs >= g_framePeriodMs) {
    g_lastTickMs = now;

    if (!loadNextFrame()) {
      ++g_sdFailStreak;
      Serial.printf("[PLAY] frame read failed — streak=%d\n", g_sdFailStreak);
      if (!recoverSd("frame read failed")) {
        if (g_sdFailStreak >= 6) {
          Serial.println("[SD] Unrecoverable — pausing playback.");
          g_playing = false;
          g_bgEffectActive = false;
          g_bgEffectNextAttemptMs = millis();
          g_sdFailStreak = 0;
          g_frameValid = false;
          blackoutAll();
        }
      }
      feedWatchdog();
      return;
    }

    g_sdFailStreak = 0;
  }

  uint32_t nowUs = micros();
  if (g_strobeEnable) {
    processHallSyncEvent(nowUs);

    const uint16_t spokeNow2 = currentSpokeIndex();
    const uint8_t arms = activeArmCount();

    const uint16_t spokes = spokesCount();
    if (spokes) {
      uint32_t base0 = (uint32_t)(g_armState[0].baseSpoke % spokes);
      uint32_t delta = (((uint32_t)spokeNow2 + (uint32_t)spokes) - base0) % (uint32_t)spokes;
      g_indexPosition = (uint16_t)((base0 + delta) % (uint32_t)spokes);
      for (uint8_t a = 0; a < arms; ++a) {
        const bool in = inStrobeWindowForArm(spokeNow2, a);
        uint32_t base = (uint32_t)(g_armState[a].baseSpoke % spokes);
        uint16_t spokeForArm = (uint16_t)((base + delta) % (uint32_t)spokes);

        if (in && g_lastPulseSpoke[a] != spokeForArm) {
          paintArmAt(a, spokeForArm, nowUs);
          g_lastPulseSpoke[a] = spokeForArm;
          g_armState[a].lit = true;
        }
      }
    }
  } else {
    processHallSyncEvent(nowUs);
    advancePredictedSpokes(nowUs);
  }

  feedWatchdog();
}

// ====== SPI/Parallel-aware blanker ======
static void blankArm(uint8_t arm){
  if (arm >= MAX_ARMS) return;

  if (g_outputMode == OUT_PARALLEL) {
    if (g_strobeEnable && PIN_STROBE_GATE >= 0) {
      g_armState[arm].lit = false;
      g_armState[arm].blankDeadlineUs = 0;
      return;
    }
    if (arm == 0) {
      configureParallelPins();
      sk9822_tx_parallel_black();
      const uint8_t arms = activeArmCount();
      for (uint8_t a=0; a<arms; ++a) { g_armState[a].lit = false; g_armState[a].blankDeadlineUs = 0; }
    }
    return;
  }

  // SPI: clear that arm's segment only
  armClear(arm);
  g_armState[arm].lit = false;
  g_armState[arm].blankDeadlineUs = 0;
}
