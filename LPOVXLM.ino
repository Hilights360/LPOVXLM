#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#define OTA_AP_SSID   "POV-OTA"
#define OTA_AP_PASS   "povupdate"

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

WebServer server(80);

static bool g_playbackEnabled = true;
static uint32_t g_frameIndex  = 0;
static uint32_t g_lastFrameMs = 0;
static String g_uploadStatus  = "Ready";

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>POV Maintenance</title>
  <style>
    body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:2rem; }
    h1 { color:#6cf; }
    section { background:#1b1b1b; padding:1rem 1.5rem; margin-bottom:1.5rem; border-radius:8px; }
    form { margin-top:0.75rem; }
    label { display:block; margin-bottom:0.5rem; }
    input[type="text"], input[type="file"] { width:100%; max-width:320px; }
    button { padding:0.5rem 1rem; border:0; border-radius:4px; background:#6cf; color:#111; font-size:1rem; cursor:pointer; }
    button:hover { background:#8df; }
    .status { margin-top:0.5rem; font-size:0.9rem; color:#9f9; }
  </style>
</head>
<body>
  <h1>POV Maintenance Portal</h1>
  <section>
    <h2>Firmware OTA Update</h2>
    <form method="post" action="/update" enctype="multipart/form-data">
      <label for="ota">Select firmware (.bin)</label>
      <input type="file" id="ota" name="update" accept=".bin" required>
      <button type="submit">Upload &amp; Flash</button>
    </form>
  </section>
  <section>
    <h2>Playback Control</h2>
    <form method="post" action="/playback/start">
      <button type="submit">Start FSEQ Playback</button>
    </form>
    <form method="post" action="/playback/stop">
      <button type="submit">Stop FSEQ Playback</button>
    </form>
  </section>
  <section>
    <h2>Upload to SD Card</h2>
    <form method="post" action="/upload-sd" enctype="multipart/form-data">
      <label for="path">Destination path on SD (e.g. /test2.fseq)</label>
      <input type="text" id="path" name="path" value="/test2.fseq" required>
      <label for="sd">Select file</label>
      <input type="file" id="sd" name="upload" required>
      <button type="submit">Upload File</button>
    </form>
    <div class="status">%STATUS%</div>
  </section>
</body>
</html>
)rawliteral";

static String expandTemplate(const String& placeholder) {
  String page = FPSTR(INDEX_HTML);
  page.replace("%STATUS%", placeholder);
  return page;
}

static void handleRoot() {
  server.send(200, "text/html", expandTemplate(g_uploadStatus));
}

static void handlePlaybackStart() {
  g_playbackEnabled = true;
  g_frameIndex = 0;
  g_lastFrameMs = millis();
  server.sendHeader("Location", "/", true);
  server.send(303);
}

static void handlePlaybackStop() {
  g_playbackEnabled = false;
  server.sendHeader("Location", "/", true);
  server.send(303);
}

static void handleOTAComplete() {
  bool ok = !Update.hasError();
  String msg = ok ? "Update Success. Rebooting..." : "Update Failed.";
  server.send(200, "text/plain", msg);
  delay(100);
  if (ok) ESP.restart();
}

static void handleOTAUpload() {
  HTTPUpload& upload = server.upload();
  switch (upload.status) {
    case UPLOAD_FILE_START:
      Serial.printf("[OTA] Update start: %s\n", upload.filename.c_str());
      if (!Update.begin()) {
        Serial.println("[OTA] Update.begin failed");
        Update.printError(Serial);
      }
      break;
    case UPLOAD_FILE_WRITE:
      if (Update.isRunning()) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Serial.println("[OTA] Update.write failed");
        }
      }
      break;
    case UPLOAD_FILE_END:
      if (Update.end(true)) {
        Serial.printf("[OTA] Update finished (%u bytes)\n", upload.totalSize);
      } else {
        Serial.print("[OTA] Update failed: ");
        Update.printError(Serial);
      }
      break;
    default:
      break;
  }
}

static File uploadFile;

static void handleSdUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (uploadFile) {
      uploadFile.close();
    }
    String path = server.arg("path");
    if (!path.length() || path[0] != '/') {
      path = "/" + path;
    }
    Serial.printf("[SD] Upload start: %s\n", path.c_str());
    uploadFile = SD_MMC.open(path, FILE_WRITE);
    if (!uploadFile) {
      g_uploadStatus = "Failed to open " + path;
      Serial.println("[SD] Open failed");
    } else {
      g_uploadStatus = "Uploading to " + path;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      if (uploadFile.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Serial.println("[SD] Write mismatch");
        g_uploadStatus = "Write error";
      }
    } else {
      g_uploadStatus = "No open file";
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      uploadFile = File();
      g_uploadStatus = "Upload complete (" + String(upload.totalSize) + " bytes)";
      Serial.println("[SD] Upload complete");
    } else {
      g_uploadStatus = "Upload failed";
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) {
      uploadFile.close();
      uploadFile = File();
    }
    g_uploadStatus = "Upload aborted";
    Serial.println("[SD] Upload aborted");
  }
}

static void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WIFI] AP '%s' started. IP: %s\n", OTA_AP_SSID, ip.toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/playback/start", HTTP_POST, handlePlaybackStart);
  server.on("/playback/stop", HTTP_POST, handlePlaybackStop);
  server.on("/update", HTTP_POST, handleOTAComplete, handleOTAUpload);
  server.on("/upload-sd", HTTP_POST, []() {
    server.sendHeader("Location", "/", true);
    server.send(303);
  }, handleSdUpload);
  server.onNotFound(handleRoot);
  server.begin();
  Serial.println("[WIFI] HTTP server ready");
}

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

  startAccessPoint();

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
  server.handleClient();

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

  if (!g_playbackEnabled) {
    delay(1);
    return;
  }

  // Timing @ 40fps
  if (millis() - g_lastFrameMs < STEP_MS) return;
  g_lastFrameMs = millis();
  if (g_frameIndex >= frames) {
    g_frameIndex = 0;
    Serial.println("[FSEQ] Looping back to frame 0");
  }

  // Read one full frame
  const uint32_t offset = chanOffset + g_frameIndex * bytesPerFrame;
  if (!fseqFile.seek(offset, fs::SeekSet)) { g_frameIndex = 0; return; }

  static uint8_t frameBuf[channels];
  const size_t n = fseqFile.read(frameBuf, bytesPerFrame);
  if (n != bytesPerFrame) { g_frameIndex = 0; return; }

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

  g_frameIndex++;
}
