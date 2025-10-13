#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

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
static const int ARM_CLK[NUM_ARMS]  = { 18, 38, 36, 48 };
static const int ARM_DATA[NUM_ARMS] = { 17, 39, 35, 45 };
// ^^^ If some pins aren’t exposed on your S3 board, swap for ones that are.
// Avoid GPIO45/46 (strap/input-only). 40/41 are known-good fast pins.

// ====== Wi-Fi OTA access point ======
#define OTA_AP_SSID   "POV-OTA"
#define OTA_AP_PASS   "povupdate"

WebServer server(80);

// ====== Web UI ======
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>POV Control</title>
  <style>
    body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:0; padding:20px; }
    h1 { margin-top:0; }
    section { background:#1d1d1d; border-radius:8px; padding:16px; margin-bottom:20px; box-shadow:0 2px 4px rgba(0,0,0,0.4); }
    button { font-size:1rem; padding:10px 18px; margin:6px 12px 6px 0; border:none; border-radius:4px; cursor:pointer; }
    button.start { background:#1e90ff; color:#fff; }
    button.stop { background:#ff6347; color:#fff; }
    button:disabled { opacity:0.5; cursor:not-allowed; }
    label { display:block; margin-bottom:8px; }
    input[type="file"] { color:#eee; }
    form { margin-top:12px; }
    #status { margin-top:12px; font-weight:bold; }
    progress { width:100%; height:20px; }
  </style>
</head>
<body>
  <h1>POV Controller</h1>
  <section>
    <h2>Playback</h2>
    <p id="playback-state">Status: <span id="playing">?</span></p>
    <button class="start" id="btn-start">Start Playback</button>
    <button class="stop" id="btn-stop">Stop Playback</button>
  </section>
  <section>
    <h2>OTA Firmware Update</h2>
    <form id="ota-form">
      <label for="ota-file">Select firmware (.bin)</label>
      <input type="file" id="ota-file" name="firmware" accept=".bin" required>
      <progress id="ota-progress" value="0" max="100" hidden></progress>
      <div id="ota-status"></div>
      <button type="submit">Upload Firmware</button>
    </form>
  </section>
  <section>
    <h2>Upload FSEQ / Assets to SD</h2>
    <form id="sd-form">
      <label for="sd-file">Select file</label>
      <input type="file" id="sd-file" name="file" required>
      <progress id="sd-progress" value="0" max="100" hidden></progress>
      <div id="sd-status"></div>
      <button type="submit">Upload to SD</button>
    </form>
  </section>
  <script>
    async function refreshStatus() {
      try {
        const res = await fetch('/status');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();
        document.getElementById('playing').textContent = data.playing ? 'Playing' : 'Stopped';
      } catch (err) {
        document.getElementById('playing').textContent = 'Error';
      }
    }

    document.getElementById('btn-start').addEventListener('click', async () => {
      await fetch('/start', { method: 'POST' });
      refreshStatus();
    });

    document.getElementById('btn-stop').addEventListener('click', async () => {
      await fetch('/stop', { method: 'POST' });
      refreshStatus();
    });

    document.getElementById('ota-form').addEventListener('submit', async (event) => {
      event.preventDefault();
      const fileInput = document.getElementById('ota-file');
      if (!fileInput.files.length) return;
      const formData = new FormData();
      formData.append('firmware', fileInput.files[0]);
      const progress = document.getElementById('ota-progress');
      const status = document.getElementById('ota-status');
      progress.hidden = false; progress.value = 0; status.textContent = 'Uploading…';
      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/update');
      xhr.upload.onprogress = (event) => {
        if (event.lengthComputable) {
          progress.value = Math.round((event.loaded / event.total) * 100);
        }
      };
      xhr.onload = () => {
        status.textContent = xhr.status === 200 ? 'Update uploaded. Device will reboot if successful.' : 'Upload failed';
        if (xhr.status === 200) progress.value = 100;
      };
      xhr.onerror = () => { status.textContent = 'Upload error'; };
      xhr.send(formData);
    });

    document.getElementById('sd-form').addEventListener('submit', async (event) => {
      event.preventDefault();
      const fileInput = document.getElementById('sd-file');
      if (!fileInput.files.length) return;
      const formData = new FormData();
      formData.append('file', fileInput.files[0]);
      const progress = document.getElementById('sd-progress');
      const status = document.getElementById('sd-status');
      progress.hidden = false; progress.value = 0; status.textContent = 'Uploading…';
      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/upload');
      xhr.upload.onprogress = (event) => {
        if (event.lengthComputable) {
          progress.value = Math.round((event.loaded / event.total) * 100);
        }
      };
      xhr.onload = () => {
        status.textContent = xhr.status === 200 ? 'File stored to SD.' : 'Upload failed';
        if (xhr.status === 200) progress.value = 100;
      };
      xhr.onerror = () => { status.textContent = 'Upload error'; };
      xhr.send(formData);
    });

    refreshStatus();
    setInterval(refreshStatus, 3000);
  </script>
</body>
</html>
)rawliteral";

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
File uploadFile;

bool g_playbackEnabled = true;
bool g_shouldReboot    = false;

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

  // Start Wi-Fi access point for OTA + control panel
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS)) {
    Serial.printf("[WIFI] AP '%s' started. IP: %s\n", OTA_AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WIFI] Failed to start AP");
  }

  // HTTP routes
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, []() {
    String json = String("{\"playing\":") + (g_playbackEnabled ? "true" : "false") + "}";
    server.send(200, "application/json", json);
  });

  server.on("/start", HTTP_POST, []() {
    g_playbackEnabled = true;
    server.send(200, "application/json", "{\"playing\":true}");
  });

  server.on("/stop", HTTP_POST, []() {
    g_playbackEnabled = false;
    server.send(200, "application/json", "{\"playing\":false}");
  });

  server.on("/upload", HTTP_POST, []() {
    if (uploadFile) {
      uploadFile.close();
      uploadFile = File();
    }
    server.send(200, "text/plain", "Upload complete");
  }, []() {
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      String path = "/" + upload.filename;
      Serial.printf("[HTTP] Upload start: %s (%u bytes expected)\n", path.c_str(), upload.totalSize);
      if (SD_MMC.exists(path)) {
        SD_MMC.remove(path);
      }
      uploadFile = SD_MMC.open(path, FILE_WRITE);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (uploadFile) {
        uploadFile.write(upload.buf, upload.currentSize);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (uploadFile) {
        uploadFile.close();
        uploadFile = File();
        Serial.printf("[HTTP] Upload complete: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
      }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      if (uploadFile) {
        uploadFile.close();
        uploadFile = File();
      }
      SD_MMC.remove(String("/") + upload.filename);
      Serial.println("[HTTP] Upload aborted");
    }
  });

  server.on("/update", HTTP_POST, []() {
    bool ok = !Update.hasError();
    server.send(ok ? 200 : 500, "text/plain", ok ? "Update successful" : "Update failed");
    if (ok) {
      g_shouldReboot = true;
    }
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[OTA] Update start: %s\n", upload.filename.c_str());
      g_playbackEnabled = false;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (!Update.end(true)) {
        Update.printError(Serial);
      } else {
        Serial.printf("[OTA] Update complete (%u bytes)\n", upload.totalSize);
      }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.end();
      Serial.println("[OTA] Update aborted");
    }
  });

  server.begin();
  Serial.println("[HTTP] Web server started on 192.168.4.1");
}

// ---------------- main loop ----------------
void loop() {
  server.handleClient();

  if (g_shouldReboot) {
    delay(1000);
    ESP.restart();
  }

  static uint32_t frameIndex = 0;
  static uint32_t last = 0;

  if (!g_playbackEnabled) {
    delay(5);
    return;
  }

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
