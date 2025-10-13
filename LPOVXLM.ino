#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ctype.h>

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

// ====== Wi-Fi AP for OTA/Web ======
#define OTA_AP_SSID   "POV-OTA"
#define OTA_AP_PASS   "povupdate"

// Arm 0 (you already wired these fast pins)
static const int ARM_CLK[NUM_ARMS]  = { 18, 38, 36, 48 };
static const int ARM_DATA[NUM_ARMS] = { 17, 39, 35, 45 };
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

// ====== Playback control ======
WebServer server(80);
bool     g_playbackEnabled = true;
uint32_t g_frameIndex      = 0;
uint32_t g_lastFrameMs     = 0;
File     g_sdUploadFile;
String   g_sdUploadPath;
bool     g_sdUploadSuccess = false;
size_t   g_sdUploadBytes   = 0;

// ====== Web UI (static HTML) ======
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>POV Control Panel</title>
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <style>
    :root { font-family: Arial, sans-serif; background:#0b0d12; color:#f2f4f8; }
    body { margin: 0 auto; padding: 24px; max-width: 640px; }
    h1 { text-align:center; }
    section { background:#141922; border-radius:12px; padding:18px 20px; margin-bottom:18px; box-shadow:0 12px 24px rgba(0,0,0,0.35); }
    button { cursor:pointer; padding:10px 18px; margin:6px 12px 6px 0; border-radius:8px; border:none; font-size:15px; font-weight:600; color:#0b0d12; background:#5df2d6; }
    button.secondary { background:#ffb86c; }
    form { display:flex; flex-wrap:wrap; align-items:center; gap:10px; }
    input[type=file] { flex:1; min-width:220px; }
    #status { margin-top:12px; font-size:0.95rem; color:#9ad0ff; }
    footer { text-align:center; font-size:0.8rem; color:#6c7a93; margin-top:32px; }
  </style>
</head>
<body>
  <h1>POV Control Panel</h1>
  <section>
    <h2>FSEQ Playback</h2>
    <form id="playback-form">
      <button type="button" data-state="start">Start Playback</button>
      <button type="button" data-state="stop" class="secondary">Stop Playback</button>
    </form>
  </section>
  <section>
    <h2>Firmware OTA Update</h2>
    <form id="ota-form" enctype="multipart/form-data">
      <input type="file" name="firmware" accept=".bin,.bin.gz" required />
      <button type="submit">Upload Firmware</button>
    </form>
  </section>
  <section>
    <h2>Upload File to SD Card</h2>
    <form id="sd-form" enctype="multipart/form-data">
      <input type="file" name="sdfile" required />
      <button type="submit">Upload File</button>
    </form>
  </section>
  <div id="status"></div>
  <footer>AP SSID: <strong>POV-OTA</strong> &bull; Password: <strong>povupdate</strong></footer>
  <script>
    async function postPlayback(state) {
      const body = new URLSearchParams();
      body.append('state', state);
      const res = await fetch('/playback', { method: 'POST', body });
      document.getElementById('status').textContent = await res.text();
    }
    document.getElementById('playback-form').addEventListener('click', (ev) => {
      if (ev.target.dataset.state) {
        postPlayback(ev.target.dataset.state);
      }
    });
    async function handleUpload(form, url) {
      const status = document.getElementById('status');
      const data = new FormData(form);
      status.textContent = 'Uploading…';
      try {
        const res = await fetch(url, { method: 'POST', body: data });
        status.textContent = await res.text();
      } catch (err) {
        status.textContent = 'Upload failed: ' + err;
      }
    }
    document.getElementById('ota-form').addEventListener('submit', (ev) => {
      ev.preventDefault();
      handleUpload(ev.target, '/ota');
    });
    document.getElementById('sd-form').addEventListener('submit', (ev) => {
      ev.preventDefault();
      handleUpload(ev.target, '/upload');
    });
  </script>
</body>
</html>
)rawliteral";

// Forward declarations
void handleRoot();
void handlePlayback();
void handleOtaUpload();
void handleSdUpload();

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handlePlayback() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Missing state parameter.");
    return;
  }
  String state = server.arg("state");
  state.toLowerCase();
  if (state == "start") {
    g_playbackEnabled = true;
    g_frameIndex      = 0;
    g_lastFrameMs     = 0;
    server.send(200, "text/plain", "Playback started.");
    Serial.println("[WEB] Playback start requested");
  } else if (state == "stop") {
    g_playbackEnabled = false;
    server.send(200, "text/plain", "Playback stopped.");
    Serial.println("[WEB] Playback stop requested");
  } else {
    server.send(400, "text/plain", "Unknown playback state.");
  }
}

void handleOtaUpload() {
  HTTPUpload& upload = server.upload();
  switch (upload.status) {
    case UPLOAD_FILE_START: {
      Serial.printf("[OTA] Update start: %s\n", upload.filename.c_str());
      g_playbackEnabled = false;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
      break;
    }
    case UPLOAD_FILE_WRITE: {
      if (Update.isRunning()) {
        const size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
          Update.printError(Serial);
        }
      }
      break;
    }
    case UPLOAD_FILE_END: {
      if (Update.end(true)) {
        Serial.printf("[OTA] Update finished (%u bytes).\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      break;
    }
    case UPLOAD_FILE_ABORTED: {
      Update.end();
      Serial.println("[OTA] Update aborted.");
      break;
    }
    default:
      break;
  }
  yield();
}

void handleSdUpload() {
  HTTPUpload& upload = server.upload();
  switch (upload.status) {
    case UPLOAD_FILE_START: {
      g_sdUploadSuccess = false;
      g_sdUploadBytes   = 0;
      g_sdUploadPath    = sanitizeUploadName(upload.filename);
      Serial.printf("[WEB] SD upload start: %s -> %s\n", upload.filename.c_str(), g_sdUploadPath.c_str());
      if (g_sdUploadFile) { g_sdUploadFile.close(); }
      if (SD_MMC.exists(g_sdUploadPath)) { SD_MMC.remove(g_sdUploadPath); }
      g_sdUploadFile = SD_MMC.open(g_sdUploadPath, FILE_WRITE);
      if (!g_sdUploadFile) {
        Serial.println("[WEB] Failed to open file on SD.");
      }
      break;
    }
    case UPLOAD_FILE_WRITE: {
      if (g_sdUploadFile) {
        const size_t written = g_sdUploadFile.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
          Serial.println("[WEB] Short write to SD file.");
        } else {
          g_sdUploadBytes += written;
        }
      }
      break;
    }
    case UPLOAD_FILE_END: {
      if (g_sdUploadFile) {
        g_sdUploadFile.close();
        g_sdUploadSuccess = true;
        Serial.printf("[WEB] SD upload complete: %s (%u bytes).\n", g_sdUploadPath.c_str(), (unsigned)g_sdUploadBytes);
      }
      break;
    }
    case UPLOAD_FILE_ABORTED: {
      if (g_sdUploadFile) {
        g_sdUploadFile.close();
      }
      if (g_sdUploadPath.length()) {
        SD_MMC.remove(g_sdUploadPath);
      }
      g_sdUploadSuccess = false;
      Serial.println("[WEB] SD upload aborted.");
      break;
    }
    default:
      break;
  }
  yield();
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

static String sanitizeUploadName(const String& name) {
  String cleaned;
  cleaned.reserve(name.length() + 2);
  for (size_t i = 0; i < name.length(); ++i) {
    const char c = name[i];
    if (isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' || c == '-') {
      cleaned += c;
    }
  }
  if (!cleaned.length()) cleaned = "upload.bin";
  if (cleaned[0] == '.') cleaned = "file" + cleaned;  // avoid hidden names
  if (cleaned[0] != '/') cleaned = String('/') + cleaned;
  return cleaned;
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n[POV] FSEQ→APA102 (4 arms, static spokes @ 0/90/180/270)");

  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS)) {
    IPAddress apIp = WiFi.softAPIP();
    Serial.printf("[WIFI] SoftAP '%s' started. IP=%s\n", OTA_AP_SSID, apIp.toString().c_str());
  } else {
    Serial.println("[WIFI] SoftAP start failed.");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/playback", HTTP_POST, handlePlayback);
  server.on("/ota", HTTP_POST,
            []() {
              if (Update.hasError()) {
                server.send(500, "text/plain", "OTA failed. Check serial log.");
              } else {
                server.send(200, "text/plain", "OTA successful. Rebooting…");
                Serial.println("[OTA] Success. Rebooting in 0.5s.");
                delay(500);
                ESP.restart();
              }
            },
            handleOtaUpload);
  server.on("/upload", HTTP_POST,
            []() {
              if (g_sdUploadSuccess) {
                String msg = "File stored as " + g_sdUploadPath + " (" + String(g_sdUploadBytes) + " bytes).";
                server.send(200, "text/plain", msg);
              } else {
                server.send(500, "text/plain", "SD upload failed. Check serial log.");
              }
              g_sdUploadPath = String();
              g_sdUploadSuccess = false;
              g_sdUploadBytes   = 0;
            },
            handleSdUpload);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("[WEB] HTTP server listening on 80");

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
  const uint32_t now = millis();
  if (now - g_lastFrameMs < STEP_MS) return;
  g_lastFrameMs = now;

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
