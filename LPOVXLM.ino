#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>

// ====== SD-MMC custom pinout (yours) ======
static const int PIN_SD_CLK = 14;
static const int PIN_SD_CMD = 15;
static const int PIN_SD_D0  = 2;
static const int PIN_SD_D1  = 4;
static const int PIN_SD_D2  = 12;
static const int PIN_SD_D3  = 13;
static const int PIN_SD_CD  = 42;     // move to 27 if 42 doesn’t read

// ====== APA102 pins per arm (EDIT THESE to match your wiring) ======
#define NUM_ARMS     4
#define LED_COUNT    144
#define STEP_MS      25               // 40 fps

// Arm 0 (you already wired these fast pins)
static const int ARM_CLK[NUM_ARMS]  = { 18, 38, 36, 11 };
static const int ARM_DATA[NUM_ARMS] = { 17, 39, 35, 10 };
// ^^^ If some pins aren’t exposed on your S3 board, swap for ones that are.
// Avoid GPIO45/46 (strap/input-only). 40/41 are known-good fast pins.

// ====== Brightness (percentage, persisted) ======
Preferences prefs;
uint8_t g_brightnessPercent = 25;  // 0–100
uint8_t g_brightness        = (255 * g_brightnessPercent) / 100;

// ====== FSEQ metadata (from your file) ======
static const uint32_t chanOffset    = 17408;   // start of frame data
static const uint32_t channels      = 17284;   // bytes per frame (all channels)
static const uint32_t frames        = 1200;    // total frames
static const uint32_t bytesPerFrame = channels;

Adafruit_DotStar* strips[NUM_ARMS] = {nullptr};
File fseqFile;

// ---------------- util ----------------
static inline bool cardPresent() {
  pinMode(PIN_SD_CD, INPUT_PULLUP);
  return digitalRead(PIN_SD_CD) == LOW;  // LOW = inserted
}

static void halt(const char* msg) {
  Serial.println(msg);
  while (true) delay(100);
}

static bool mountSdmmc() {
  Serial.print("[SD_MMC] setPins (4-bit)… ");
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
  Serial.println("OK");
  Serial.print("[SD_MMC] begin (4-bit)… ");
  if (SD_MMC.begin("/sdcard", false, false)) { Serial.println("OK (4-bit)"); return true; }
  Serial.println("FAIL (4-bit)");
  Serial.print("[SD_MMC] begin (1-bit fallback)… ");
  if (SD_MMC.begin("/sdcard", true, false))  { Serial.println("OK (1-bit)"); return true; }
  Serial.println("FAIL (1-bit)");
  return false;
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n[POV] FSEQ→APA102 (4 arms, static spokes @ 0/90/180/270)");

  // Restore brightness % from NVS
  prefs.begin("display", false);
  g_brightnessPercent = prefs.getUChar("brightness", 25);
  g_brightness        = (255 * g_brightnessPercent) / 100;
  Serial.printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);

  if (!cardPresent()) {
    Serial.printf("[SD_MMC] Card-detect HIGH on GPIO%d (no card)\n", PIN_SD_CD);
    halt("HALT");
  }
  if (!mountSdmmc()) halt("[SD_MMC] Mount failed");

  uint8_t type = SD_MMC.cardType();
  if (type == CARD_NONE) halt("[SD_MMC] No card");
  Serial.printf("[SD_MMC] Type=%u  Size=%llu MB\n",
                (unsigned)type, (unsigned long long)(SD_MMC.cardSize() / (1024ULL*1024ULL)));

  fseqFile = SD_MMC.open("/test2.fseq", FILE_READ);
  if (!fseqFile) halt("[SD_MMC] Cannot open /test2.fseq");

  const uint64_t fileSize = fseqFile.size();
  const uint64_t payload  = (fileSize > chanOffset) ? (fileSize - chanOffset) : 0ULL;
  Serial.printf("[FSEQ] size=%llu  payload=%llu  bytesPerFrame=%lu  frames=%lu\n",
                (unsigned long long)fileSize, (unsigned long long)payload,
                (unsigned long)bytesPerFrame, (unsigned long)frames);

  // Create and init 4 DotStar strips
  for (int a = 0; a < NUM_ARMS; ++a) {
    strips[a] = new Adafruit_DotStar(LED_COUNT, ARM_DATA[a], ARM_CLK[a], DOTSTAR_BGR);
    strips[a]->begin();
    strips[a]->setBrightness(g_brightness);
    strips[a]->show();
  }
  Serial.println("[LED] 4 arms ready");
  Serial.println("Type 'Bxx' (e.g. B25) to set brightness percent (0–100).");
}

// ---------------- main loop ----------------
void loop() {
  static uint32_t frameIndex = 0;
  static uint32_t last = 0;

  // Serial brightness command (e.g. B10 for 10%)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() && (cmd[0] == 'B' || cmd[0] == 'b')) {
      int pct = cmd.substring(1).toInt();
      if (pct < 0)   pct = 0;
      if (pct > 100) pct = 100;
      g_brightnessPercent = pct;
      g_brightness = (255 * g_brightnessPercent) / 100;
      for (int a = 0; a < NUM_ARMS; ++a) { strips[a]->setBrightness(g_brightness); }
      for (int a = 0; a < NUM_ARMS; ++a) { strips[a]->show(); } // apply immediately
      prefs.putUChar("brightness", g_brightnessPercent);
      Serial.printf("[LED] Brightness %u%% (%u) saved\n", g_brightnessPercent, g_brightness);
    }
  }

  // Timing @ 40fps
  if (millis() - last < STEP_MS) return;
  last = millis();
  if (frameIndex >= frames) {
    frameIndex = 0;
    Serial.println("[FSEQ] Looping back to frame 0");
  }

  // Read one full frame
  const uint32_t offset = chanOffset + frameIndex * bytesPerFrame;
  if (!fseqFile.seek(offset, fs::SeekSet)) { frameIndex = 0; return; }

  static uint8_t frameBuf[channels];
  const size_t n = fseqFile.read(frameBuf, bytesPerFrame);
  if (n != bytesPerFrame) { frameIndex = 0; return; }

  // Determine arms per revolution in the file
  const uint32_t armSize   = LED_COUNT * 3;                   // bytes per logical arm
  const uint32_t totalArms = bytesPerFrame / armSize;         // e.g. 40
  if (totalArms == 0) return;

  // Choose four static source spokes 90° apart:
  // 0°, 90°, 180°, 270°  => arm indices {0, totalArms/4, totalArms/2, 3*totalArms/4}
  const uint32_t quarter = totalArms / 4;                     // integer divide
  const uint32_t srcIdx[NUM_ARMS] = {
    0,
    (quarter) % totalArms,
    (2*quarter) % totalArms,
    (3*quarter) % totalArms
  };

  // For each physical arm, copy that arm’s slice to its strip
  for (int a = 0; a < NUM_ARMS; ++a) {
    const uint32_t armOff = srcIdx[a] * armSize;
    if (armOff + armSize > bytesPerFrame) continue;           // guard

    for (uint32_t i = 0; i < LED_COUNT; ++i) {
      const uint32_t p = armOff + i*3;
      const uint8_t r = frameBuf[p + 0];
      const uint8_t g = frameBuf[p + 1];
      const uint8_t b = frameBuf[p + 2];
      strips[a]->setPixelColor(i, r, g, b);
    }
  }
  // Latch all arms together
  for (int a = 0; a < NUM_ARMS; ++a) { strips[a]->show(); }

  frameIndex++;
}
