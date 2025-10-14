#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>

// ===== Wi-Fi AP + Web UI =====
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ===== SD-MMC (1-bit per your current wiring) =====
static const int PIN_SD_CLK = 14;
static const int PIN_SD_CMD = 15;
static const int PIN_SD_D0  = 2;
// Keep these for an easy switch back to 4-bit later:
static const int PIN_SD_D1  = 4;
static const int PIN_SD_D2  = 12;
static const int PIN_SD_D3  = 13;
static const int PIN_SD_CD  = 42;     // LOW = inserted (invert if needed)

// ===== APA102 pins per arm =====
#define NUM_ARMS     4
#define LED_COUNT    144

// ===== Playback timing (now variable FPS) =====
Preferences prefs;
uint16_t g_fps = 40;                  // adjustable 1..120
uint32_t g_framePeriodMs = 1000 / 40; // derived from g_fps

// ===== FSEQ timing =====
static const uint32_t STEP_MS_DEFAULT = 25; // legacy reference
// ===== FSEQ metadata (static for now) =====
static const uint32_t chanOffset    = 17408;   // start of frame data
static const uint32_t channels      = 17284;   // bytes per frame (all channels)
static const uint32_t frames        = 1200;    // total frames in file
static const uint32_t bytesPerFrame = channels;

// NOTE: Some ESP32-S3 modules have GPIO45 input-only.
static const int ARM_CLK[NUM_ARMS]  = { 18, 38, 36, 48 };
static const int ARM_DATA[NUM_ARMS] = { 17, 39, 35, 45 };

Adafruit_DotStar* strips[NUM_ARMS] = {nullptr};
File fseqFile;

// ===== Brightness (percentage, persisted) =====
uint8_t g_brightnessPercent = 25;  // 0–100
uint8_t g_brightness        = (255 * g_brightnessPercent) / 100;

// ===== Playback state =====
volatile bool g_playing   = false;  // "a sequence is selected"
volatile bool g_paused    = false;  // "auto advance disabled"
String   g_currentPath    = "";     // e.g. "/test2.fseq"
uint32_t g_frameIndex     = 0;      // NEXT frame to render
uint32_t g_lastTickMs     = 0;

// ===== 5-minute selection timeout =====
const uint32_t SELECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;
uint32_t g_bootMs = 0;

// ===== Wi-Fi AP config =====
static const char* AP_SSID  = "POV-Spinner";
static const char* AP_PASS  = "POV123456";   // 8+ chars
static const IPAddress AP_IP(192,168,4,1);
static const IPAddress AP_GW(192,168,4,1);
static const IPAddress AP_MASK(255,255,255,0);

WebServer server(80);

// ===== Upload state (streaming to SD) =====
File   g_uploadFile;
String g_uploadFilename;
size_t g_uploadBytes = 0;

// ===== Debug & mapping =====
// Per-arm whole-direction (global reverse)
bool g_armReverse[NUM_ARMS] = { false, false, false, false };

// Color channel mapping: file buffer -> R,G,B we send to DotStar
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

// ----- Status helpers (UI) -----
static inline const char* statusText() {
  if (g_paused && g_playing) return "Paused";
  if (g_playing)             return "Playing";
  return "Stopped";
}
static inline const char* statusClass() {
  if (g_paused && g_playing) return "badge pause";
  if (g_playing)             return "badge play";
  return "badge stop";
}

// ---- Per-arm split remap ----
// For each arm: indices [0..split-1] = Segment A, [split..LED_COUNT-1] = Segment B
// Each segment can be normal (0) or reversed (1) relative to logical inside->outside.
uint16_t g_armSplit[NUM_ARMS]   = { LED_COUNT, LED_COUNT, LED_COUNT, LED_COUNT }; // LED_COUNT => no split
bool     g_segRevA[NUM_ARMS]    = { false, false, false, false };
bool     g_segRevB[NUM_ARMS]    = { false, false, false, false };

// Map logical LED index 'i' (inside->outside) to physical index with split + segments + whole-arm reverse.
static inline uint32_t mapIndexWithSplit(int arm, uint32_t i) {
  uint16_t split = g_armSplit[arm];
  if (split > LED_COUNT) split = LED_COUNT;

  const bool armRev = g_armReverse[arm];

  if (i < split) {
    uint32_t lenA = split;
    uint32_t idxA = g_segRevA[arm] ? (lenA - 1 - i) : i;
    return armRev ? (LED_COUNT - 1 - idxA) : idxA;
  } else {
    uint32_t lenB = LED_COUNT - split;
    uint32_t j    = i - split;
    uint32_t idxB = g_segRevB[arm] ? (lenB - 1 - j) : j;
    uint32_t phys = split + idxB;
    return armRev ? (LED_COUNT - 1 - phys) : phys;
  }
}

// ---------------- util ----------------
static inline bool cardPresent() {
  pinMode(PIN_SD_CD, INPUT_PULLUP);
  return digitalRead(PIN_SD_CD) == LOW;  // LOW = inserted
}

static bool mountSdmmc() {
  Serial.print("[SD_MMC] setPins (custom)… ");
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
  Serial.println("OK");

  const uint32_t FREQ_KHZ = 10000; // 10 MHz
  Serial.printf("[SD_MMC] begin (1-bit, %u kHz)… ", (unsigned)FREQ_KHZ);
  if (SD_MMC.begin("/sdcard", true /*1-bit*/, false /*no format*/, FREQ_KHZ)) {
    Serial.println("OK");
    return true;
  }
  Serial.println("FAIL");
  return false;
}

static void blackoutAll() {
  for (int a = 0; a < NUM_ARMS; ++a) {
    if (!strips[a]) continue;
    for (uint32_t i = 0; i < LED_COUNT; ++i) strips[a]->setPixelColor(i, 0, 0, 0);
    strips[a]->show();
  }
}

static void applyBrightness(uint8_t pct) {
  if (pct > 100) pct = 100;
  g_brightnessPercent = pct;
  g_brightness = (uint8_t)((255 * g_brightnessPercent) / 100);
  for (int a = 0; a < NUM_ARMS; ++a) if (strips[a]) strips[a]->setBrightness(g_brightness);
  for (int a = 0; a < NUM_ARMS; ++a) if (strips[a]) strips[a]->show();
  prefs.putUChar("brightness", g_brightnessPercent);
  Serial.printf("[LED] Brightness %u%% (%u) saved\n", g_brightnessPercent, g_brightness);
}

static bool isFseqName(const String& n) {
  int dot = n.lastIndexOf('.');
  if (dot < 0) return false;
  String ext = n.substring(dot + 1);
  ext.toLowerCase();
  return ext == "fseq";
}

static void listFseqInDir(const char* path, String& optionsHtml, uint8_t depth = 0) {
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
  File ent;
  while ((ent = dir.openNextFile())) {
    String name = ent.name();
    if (ent.isDirectory()) {
      if (depth == 0) listFseqInDir(name.c_str(), optionsHtml, depth + 1);
    } else {
      if (isFseqName(name)) {
        optionsHtml += "<option value='";
        optionsHtml += name;
        optionsHtml += "'";
        if (name == g_currentPath) optionsHtml += " selected";
        optionsHtml += ">";
        optionsHtml += name;
        optionsHtml += "</option>";
      }
    }
    ent.close();
  }
  dir.close();
}

static bool openFseq(const String& path) {
  if (fseqFile) fseqFile.close();
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { Serial.printf("[FSEQ] Open failed: %s\n", path.c_str()); return false; }
  fseqFile = f;
  g_currentPath = path;
  g_frameIndex = 0;                 // next to render
  g_playing = true;
  g_lastTickMs = millis();
  Serial.printf("[FSEQ] Playing: %s\n", g_currentPath.c_str());

  const uint64_t fileSize = fseqFile.size();
  const uint64_t payload  = (fileSize > chanOffset) ? (fileSize - chanOffset) : 0ULL;
  Serial.printf("[FSEQ] size=%llu payload=%llu bytesPerFrame=%lu frames(const)=%lu\n",
    (unsigned long long)fileSize, (unsigned long long)payload,
    (unsigned long)bytesPerFrame, (unsigned long)frames);
  return true;
}

static void stopPlayback() {
  g_playing = false;
  if (fseqFile) fseqFile.close();
  g_currentPath = "";
  blackoutAll();
  Serial.println("[FSEQ] Stopped, LEDs blacked out");
}

// ======= RENDERING CORE =======
static uint8_t frameBuf[channels]; // persistent buffer

// Render EXACTLY the frame at index 'idx' and LATCH.
// Does not consider g_paused; pure render.
static bool renderFrame(uint32_t idx) {
  if (!fseqFile) return false;
  if (frames == 0) return false;

  idx %= frames;
  const uint32_t offset = chanOffset + idx * bytesPerFrame;
  if (!fseqFile.seek(offset, fs::SeekSet)) {
    Serial.println("[FSEQ] seek fail"); return false;
  }
  const size_t n = fseqFile.read(frameBuf, bytesPerFrame);
  if (n != bytesPerFrame) {
    Serial.println("[FSEQ] short read"); return false;
  }

  // Determine arms per revolution in the file
  const uint32_t armSize   = LED_COUNT * 3;            // bytes per logical arm
  const uint32_t totalArms = bytesPerFrame / armSize;  // e.g. 40
  if (totalArms == 0) return false;

  // Choose four static source spokes 90° apart
  const uint32_t quarter = totalArms / 4;
  const uint32_t srcIdx[NUM_ARMS] = {
    0, quarter % totalArms, (2*quarter) % totalArms, (3*quarter) % totalArms
  };

  // Paint each arm from the chosen source slice
  for (int a = 0; a < NUM_ARMS; ++a) {
    const uint32_t armOff = srcIdx[a] * armSize;
    if (armOff + armSize > bytesPerFrame) continue;

    for (uint32_t i = 0; i < LED_COUNT; ++i) {
      const uint32_t p = armOff + i*3;
      uint8_t R,G,B; mapChannels(&frameBuf[p], R,G,B);
      uint32_t phys = mapIndexWithSplit(a, i);
      strips[a]->setPixelColor(phys, R, G, B);
    }
  }
  for (int a = 0; a < NUM_ARMS; ++a) strips[a]->show();
  return true;
}

// Auto-advance scheduler: if playing and not paused, render next frame when due
static void tickPlayer() {
  if (!g_playing || g_paused) return;

  const uint32_t now = millis();
  if (now - g_lastTickMs < g_framePeriodMs) return;
  g_lastTickMs = now;

  if (!renderFrame(g_frameIndex)) { g_frameIndex = 0; return; }
  g_frameIndex = (g_frameIndex + 1) % frames;
}

// ---------------- Web handlers ----------------
static void handleStatus() {
  String cur = g_currentPath.length() ? g_currentPath : "";
  String json = String("{\"playing\":") + (g_playing ? "true":"false")
              + ",\"paused\":" + (g_paused ? "true":"false")
              + ",\"text\":\"" + statusText() + "\""
              + ",\"path\":\"" + cur + "\""
              + ",\"frame\":" + g_frameIndex
              + ",\"fps\":" + g_fps + "}";
  server.send(200, "application/json", json);
}

static void handleRoot() {
  String options; listFseqInDir("/", options);
  String cur = g_currentPath.length() ? g_currentPath : "(none)";

  String html =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>POV Spinner</title>"
    "<style>"
    "body{font:16px system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0b1320;color:#e8ecf1;margin:0;padding:1rem}"
    ".card{max-width:760px;margin:0 auto;background:#121b2d;padding:1rem;border-radius:12px;box-shadow:0 1px 8px rgba(0,0,0,.2)}"
    "label{display:block;margin:.5rem 0 .2rem}select,input[type=range],input[type=number]{width:100%}"
    ".row{display:flex;gap:.5rem;flex-wrap:wrap;margin-top:.5rem}"
    "button{padding:.6rem 1rem;border:0;border-radius:10px;background:#1c2b4a;color:#e8ecf1;cursor:pointer}"
    "button:hover{filter:brightness(1.1)}"
    "input[type=file]{width:100%;padding:.5rem;border-radius:10px;border:1px solid #253756;background:#0e1627;color:#e8ecf1}"
    ".muted{opacity:.75}"
    ".pill{display:inline-block;padding:.2rem .6rem;border-radius:999px;background:#0e1627;margin-left:.5rem}"
    ".sep{height:1px;background:#1b2741;margin:1rem 0}"
    ".badge{display:inline-block;margin-left:.5rem;padding:.15rem .55rem;border-radius:999px;font-size:.85rem}"
    ".badge.play{background:#0e2a19;color:#9af0b7}"
    ".badge.pause{background:#2a1f0e;color:#f0d49a}"
    ".badge.stop{background:#2a0e12;color:#f09aa6}"
    "</style></head><body>"
    "<div class='card'>"
      "<h1 style='display:flex;align-items:center;gap:.4rem'>"
        "POV Spinner"
        "<span id='status' class='" + String(statusClass()) + "'>" + String(statusText()) + "</span>"
        "<span id='which' class='pill'>" + cur + "</span>"
      "</h1>"
      "<p>AP SSID: <b>POV-Spinner</b> &middot; IP: <b>192.168.4.1</b> &middot; mDNS: <b>pov.local</b></p>"

      "<label>Choose .fseq file</label>"
      "<select id='sel'>" + options + "</select>"
      "<div class='row'>"
        "<button id='start'>Start</button>"
        "<button id='stop'>Stop</button>"
        "<button id='refresh'>Refresh</button>"
      "</div>"

      "<div class='sep'></div>"
      "<h3>Upload a new .fseq to SD</h3>"
      "<form id='u' method='POST' action='/upload' enctype='multipart/form-data'>"
        "<input type='file' name='f' accept='.fseq' required>"
        "<div class='row'><button type='submit'>Upload</button></div>"
        "<p class='muted'>Uploads go to the SD card root. Large files stream directly to SD.</p>"
      "</form>"

      "<div class='sep'></div>"
      "<h3>Playback Speed</h3>"
      "<label>FPS: <span id='fpsv'>" + String(g_fps) + "</span></label>"
      "<input id='fps' type='range' min='1' max='120' value='" + String(g_fps) + "'>"
      "<div class='row'><button id='applyfps'>Apply</button><button id='fps10'>10 FPS</button><button id='fps40'>40 FPS</button><button id='fps60'>60 FPS</button></div>"

      "<div class='sep'></div>"
      "<h3>Brightness</h3>"
      "<label>Value: <span id='v'>" + String(g_brightnessPercent) + "%</span></label>"
      "<input id='rng' type='range' min='0' max='100' value='" + String(g_brightnessPercent) + "'>"
      "<div class='row'><button id='set'>Apply</button><button id='low'>10%</button><button id='med'>40%</button><button id='hi'>100%</button></div>"

      "<div class='sep'></div>"
      "<h3>Debug</h3>"
      "<div class='row'>"
        "<button id='pause'>Pause</button>"
        "<button id='stepb'>Step −</button>"
        "<button id='stepf'>Step +</button>"
        "<button id='resume'>Resume</button>"
        "<button onclick=\"post('/order?m='+prompt('Order RGB/RBG/GBR/GRB/BRG/BGR','RGB'))\">Color Order…</button>"
      "</div>"
      "<div class='row' style='margin-top:.5rem'>"
        "<button onclick=\"post('/rev?a=0&on='+(prompt('Reverse arm 0? 0/1','1')||'1'))\">Toggle Arm0 Rev</button>"
        "<button onclick=\"post('/rev?a=1&on='+(prompt('Reverse arm 1? 0/1','1')||'1'))\">Toggle Arm1 Rev</button>"
        "<button onclick=\"post('/rev?a=2&on='+(prompt('Reverse arm 2? 0/1','1')||'1'))\">Toggle Arm2 Rev</button>"
        "<button onclick=\"post('/rev?a=3&on='+(prompt('Reverse arm 3? 0/1','1')||'1'))\">Toggle Arm3 Rev</button>"
      "</div>"
      "<div class='row' style='margin-top:.5rem'>"
        "<button onclick='setMap(0)'>Map Arm0</button>"
        "<button onclick='setMap(1)'>Map Arm1</button>"
        "<button onclick='setMap(2)'>Map Arm2</button>"
        "<button onclick='setMap(3)'>Map Arm3</button>"
      "</div>"

      "<div class='sep'></div>"
      "<p class='muted'>If no file is started within 5 minutes after boot, <b>/test2.fseq</b> will auto-play.</p>"
    "</div>"

    "<script>"
    "const LEDS=" + String(LED_COUNT) + ";"
    "const fps=document.getElementById('fps'), fpsv=document.getElementById('fpsv');"
    "fps.oninput=()=>fpsv.textContent=fps.value;"
    "const r=document.getElementById('rng'),v=document.getElementById('v');"
    "r.oninput=()=>v.textContent=r.value+'%';"
    "function post(u){fetch(u,{method:'POST'}).then(()=>location.reload());}"
    "document.getElementById('applyfps').onclick=()=>post('/speed?fps='+fps.value);"
    "document.getElementById('fps10').onclick=()=>post('/speed?fps=10');"
    "document.getElementById('fps40').onclick=()=>post('/speed?fps=40');"
    "document.getElementById('fps60').onclick=()=>post('/speed?fps=60');"

    "document.getElementById('set').onclick=()=>post('/b?pct='+r.value);"
    "document.getElementById('low').onclick=()=>post('/b?pct=10');"
    "document.getElementById('med').onclick=()=>post('/b?pct=40');"
    "document.getElementById('hi').onclick=()=>post('/b?pct=100');"
    "document.getElementById('start').onclick=()=>{const p=document.getElementById('sel').value;fetch('/start?path='+encodeURIComponent(p)).then(()=>location.reload());};"
    "document.getElementById('stop').onclick =()=>post('/stop');"
    "document.getElementById('refresh').onclick=()=>location.reload();"
    "document.getElementById('pause').onclick =()=>post('/pause');"
    "document.getElementById('resume').onclick=()=>post('/resume');"
    "document.getElementById('stepf').onclick=()=>post('/step?dir=1');"
    "document.getElementById('stepb').onclick=()=>post('/step?dir=-1');"

    // Split mapper helper
    "function setMap(a){"
      "const s = prompt('Arm '+a+' split index (0..'+LEDS+')',''+(LEDS-16));"
      "if (s===null) return;"
      "const ra = prompt('Segment A reversed? 0/1','0');"
      "if (ra===null) return;"
      "const rb = prompt('Segment B reversed? 0/1','1');"
      "if (rb===null) return;"
      "fetch('/armmap?a='+a+'&split='+encodeURIComponent(s)+'&ra='+encodeURIComponent(ra)+'&rb='+encodeURIComponent(rb),{method:'POST'})"
        ".then(()=>location.reload());"
    "}"

    // Live status + which sequence updater every 1s
    "function upd(){fetch('/status').then(r=>r.json()).then(s=>{"
      "const el=document.getElementById('status');"
      "el.textContent=s.text;"
      "el.className='badge '+(s.paused&&s.playing?'pause':(s.playing?'play':'stop'));"
      "document.getElementById('which').textContent = s.path && s.path.length ? s.path : '(none)';"
      "fps.value=s.fps; fpsv.textContent=s.fps;"
    "});}"
    "setInterval(upd,1000);"
    "</script>"
    "</body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

static void handleSetBrightness() {
  if (!server.hasArg("pct")) { server.send(400, "text/plain", "missing pct"); return; }
  int pct = server.arg("pct").toInt();
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  applyBrightness((uint8_t)pct);
  server.send(200, "application/json", String("{\"brightness\":") + pct + "}");
}

static void handleStart() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  if (!SD_MMC.exists(path)) { server.send(404, "text/plain", "file not found"); return; }
  if (!isFseqName(path))    { server.send(415, "text/plain", "unsupported (need .fseq)"); return; }
  if (!openFseq(path)) { server.send(500, "text/plain", "open failed"); return; }
  server.send(200, "application/json", String("{\"started\":\"") + path + "\"}");
}

static void handleStop() { stopPlayback(); server.send(200, "application/json", "{\"stopped\":true}"); }

// --- Upload: data stream handler (called repeatedly during POST) ---
static void handleUploadData() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_uploadBytes = 0;
    g_uploadFilename = up.filename;
    if (!g_uploadFilename.length()) g_uploadFilename = "upload.fseq";
    int slash = g_uploadFilename.lastIndexOf('/');
    if (slash >= 0) g_uploadFilename = g_uploadFilename.substring(slash + 1);
    if (!g_uploadFilename.startsWith("/")) g_uploadFilename = "/" + g_uploadFilename;

    if (!isFseqName(g_uploadFilename)) {
      Serial.printf("[UPLOAD] Rejected non-.fseq: %s\n", g_uploadFilename.c_str());
    } else {
      if (SD_MMC.exists(g_uploadFilename)) SD_MMC.remove(g_uploadFilename);
      g_uploadFile = SD_MMC.open(g_uploadFilename, FILE_WRITE);
      Serial.printf("[UPLOAD] START %s\n", g_uploadFilename.c_str());
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_uploadFile) { g_uploadFile.write(up.buf, up.currentSize); g_uploadBytes += up.currentSize; }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_uploadFile) { g_uploadFile.close(); Serial.printf("[UPLOAD] DONE %s (%u bytes)\n", g_uploadFilename.c_str(), (unsigned)g_uploadBytes); }
    else { Serial.println("[UPLOAD] Aborted/invalid file"); }
  }
}

// --- Upload: finalizer (called once after upload completes) ---
static void handleUploadDone() {
  if (!isFseqName(g_uploadFilename)) {
    server.send(415, "text/html", "<meta http-equiv='refresh' content='2;url=/'><body style='font-family:system-ui'><p>Upload rejected. Only <b>.fseq</b> files are allowed.</p><p>Returning…</p></body>");
    return;
  }
  if (!SD_MMC.exists(g_uploadFilename)) {
    server.send(500, "text/html", "<meta http-equiv='refresh' content='3;url=/'><body style='font-family:system-ui'><p>Upload failed.</p><p>Returning…</p></body>");
    return;
  }
  server.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/'><body style='font-family:system-ui'><p>Uploaded <b>" + g_uploadFilename + "</b> (" + String(g_uploadBytes) + " bytes).</p><p>Refreshing…</p></body>");
}

// --- Playback control: pause/resume/step/speed ---
static void handlePause()  { g_paused = true;  server.send(200, "application/json", "{\"paused\":true}"); }
static void handleResume() { g_paused = false; g_lastTickMs = millis(); server.send(200, "application/json", "{\"paused\":false}"); }

// /step?dir=1  (forward)  or /step?dir=-1 (backward)
static void handleStep() {
  int dir = 1;
  if (server.hasArg("dir")) dir = server.arg("dir").toInt();
  if (frames == 0 || !fseqFile) { server.send(409, "application/json", "{\"ok\":false}"); return; }

  g_paused = true; // step is meaningful in paused mode
  if (dir >= 0) {
    // Render the frame pointed by g_frameIndex, then advance
    if (!renderFrame(g_frameIndex)) { server.send(500, "application/json", "{\"ok\":false}"); return; }
    g_frameIndex = (g_frameIndex + 1) % frames;
  } else {
    // Render previous frame, then leave g_frameIndex pointing to the original "next"
    uint32_t prev = (g_frameIndex + frames - 1) % frames;
    if (!renderFrame(prev)) { server.send(500, "application/json", "{\"ok\":false}"); return; }
    // Keep g_frameIndex unchanged, so another step+ shows the “current” again
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

// /speed?fps=NN
static void handleSpeed() {
  if (!server.hasArg("fps")) { server.send(400, "text/plain", "missing fps"); return; }
  int val = server.arg("fps").toInt();
  if (val < 1) val = 1;
  if (val > 120) val = 120;
  g_fps = (uint16_t)val;
  g_framePeriodMs = (uint32_t) (1000UL / g_fps);
  prefs.putUShort("fps", g_fps);
  g_lastTickMs = millis(); // reset scheduler
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
  server.send(200, "application/json", String("{\"fps\":") + g_fps + "}");
}

// --- Reverse per arm /rev?a=0..3&on=0/1 ---
static void handleReverseArm() {
  if (!server.hasArg("a") || !server.hasArg("on")) { server.send(400, "text/plain", "args"); return; }
  int a = server.arg("a").toInt(); int on = server.arg("on").toInt();
  if (a < 0 || a >= NUM_ARMS) { server.send(400, "text/plain", "bad arm"); return; }
  g_armReverse[a] = (on != 0);
  server.send(200, "application/json", String("{\"arm\":") + a + ",\"reverse\":" + (g_armReverse[a]?"true":"false") + "}");
}

// --- Color order /order?m=RGB|RBG|GBR|GRB|BRG|BGR ---
static void handleOrder() {
  if (!server.hasArg("m")) { server.send(400, "text/plain", "missing m"); return; }
  String m = server.arg("m"); m.toUpperCase();
  if      (m=="RGB") g_colorMap = MAP_RGB;
  else if (m=="RBG") g_colorMap = MAP_RBG;
  else if (m=="GBR") g_colorMap = MAP_GBR;
  else if (m=="GRB") g_colorMap = MAP_GRB;
  else if (m=="BRG") g_colorMap = MAP_BRG;
  else if (m=="BGR") g_colorMap = MAP_BGR;
  else { server.send(400, "text/plain", "bad map"); return; }
  server.send(200, "application/json", String("{\"order\":\"") + m + "\"}");
}

// --- Peek raw bytes for a given arm/index in CURRENT "next" frame ---
static void handlePeek() {
  if (!server.hasArg("arm") || !server.hasArg("i")) { server.send(400, "text/plain", "args"); return; }
  int arm = server.arg("arm").toInt(); int i = server.arg("i").toInt();
  if (arm < 0 || arm >= NUM_ARMS || i < 0 || i >= LED_COUNT) { server.send(400, "text/plain", "range"); return; }

  const uint32_t armSize   = LED_COUNT * 3;
  const uint32_t totalArms = bytesPerFrame / armSize;
  if (!totalArms) { server.send(500, "text/plain", "totalArms=0"); return; }
  const uint32_t quarter = totalArms / 4;
  const uint32_t srcIdx[NUM_ARMS] = { 0, quarter % totalArms, (2*quarter)%totalArms, (3*quarter)%totalArms };

  uint32_t armOff = srcIdx[arm] * armSize;
  uint32_t p = armOff + (uint32_t)i * 3;

  if (!fseqFile) { server.send(409, "text/plain", "no file"); return; }
  const uint32_t idx = (g_frameIndex % frames);
  const uint32_t offset = chanOffset + idx * bytesPerFrame + p;
  if (!fseqFile.seek(offset, fs::SeekSet)) { server.send(500, "text/plain", "seek fail"); return; }

  uint8_t raw[3] = {0,0,0};
  size_t n = fseqFile.read(raw, 3);
  if (n != 3) { server.send(500, "text/plain", "read fail"); return; }

  uint8_t R,G,B; mapChannels(raw, R,G,B);
  String json = String("{\"nextFrame\":") + idx +
    ",\"arm\":" + arm + ",\"i\":" + i +
    ",\"raw\":[" + raw[0] + "," + raw[1] + "," + raw[2] + "]," +
    "\"mapped\":[" + R + "," + G + "," + B + "]," +
    "\"reverse\":" + (g_armReverse[arm]?"true":"false") + "}";
  server.send(200, "application/json", json);
}

// --- Split remap endpoints ---
// GET /armmap?a=0  -> {split, ra, rb, armrev}
static void handleGetArmMap() {
  if (!server.hasArg("a")) { server.send(400, "text/plain", "need a"); return; }
  int a = server.arg("a").toInt();
  if (a < 0 || a >= NUM_ARMS) { server.send(400, "text/plain", "bad arm"); return; }

  String json = String("{\"arm\":") + a +
    ",\"split\":" + g_armSplit[a] +
    ",\"ra\":" + (g_segRevA[a] ? "true" : "false") +
    ",\"rb\":" + (g_segRevB[a] ? "true" : "false") +
    ",\"armrev\":" + (g_armReverse[a] ? "true" : "false") +
    "}";
  server.send(200, "application/json", json);
}

// POST /armmap?a=0&split=130&ra=0&rb=1
static void handleSetArmMap() {
  if (!server.hasArg("a")) { server.send(400, "text/plain", "need a"); return; }
  int a = server.arg("a").toInt();
  if (a < 0 || a >= NUM_ARMS) { server.send(400, "text/plain", "bad arm"); return; }

  if (server.hasArg("split")) {
    int s = server.arg("split").toInt();
    if (s < 0) s = 0; if (s > (int)LED_COUNT) s = LED_COUNT;
    g_armSplit[a] = (uint16_t)s;
  }
  if (server.hasArg("ra")) g_segRevA[a] = (server.arg("ra").toInt() != 0);
  if (server.hasArg("rb")) g_segRevB[a] = (server.arg("rb").toInt() != 0);

  handleGetArmMap();
}

static void handleNotFound() { server.send(404, "text/plain", "Not found"); }

static void startWifiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  Serial.printf("[AP] softAP %s (%s)\n", ok ? "started" : "FAILED", AP_SSID);
  Serial.printf("[AP] IP: %s\n", WiFi.softAPIP().toString().c_str());

  if (MDNS.begin("pov")) { MDNS.addService("http", "tcp", 80); Serial.println("[mDNS] pov.local advertised"); }
  else { Serial.println("[mDNS] start failed"); }

  // Routes
  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/status",  HTTP_GET,  handleStatus);
  server.on("/b",       HTTP_POST, handleSetBrightness);
  server.on("/start",   HTTP_GET,  handleStart);
  server.on("/stop",    HTTP_POST, handleStop);
  server.on("/upload",  HTTP_POST, handleUploadDone, handleUploadData);

  // Playback control
  server.on("/pause",   HTTP_POST, handlePause);
  server.on("/resume",  HTTP_POST, handleResume);
  server.on("/step",    HTTP_POST, handleStep);
  server.on("/speed",   HTTP_POST, handleSpeed);

  // Debug routes
  server.on("/rev",     HTTP_POST, handleReverseArm);
  server.on("/order",   HTTP_POST, handleOrder);
  server.on("/peek",    HTTP_GET,  handlePeek);

  // Split remap routes
  server.on("/armmap",  HTTP_GET,  handleGetArmMap);
  server.on("/armmap",  HTTP_POST, handleSetArmMap);

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] WebServer listening on :80");
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n[POV] FSEQ→APA102 (4 arms, static spokes @ 0/90/180/270)");

  // Restore settings
  prefs.begin("display", false);
  g_brightnessPercent = prefs.getUChar("brightness", 25);
  g_brightness        = (255 * g_brightnessPercent) / 100;
  g_fps               = prefs.getUShort("fps", 40);
  if (g_fps < 1) g_fps = 1; if (g_fps > 120) g_fps = 120;
  g_framePeriodMs     = 1000UL / g_fps;

  Serial.printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);

  // Bring up AP early
  startWifiAP();

  // SD card
  if (!cardPresent()) {
    Serial.printf("[SD_MMC] Card-detect indicates no card on GPIO%d; continuing for UI/diag\n", PIN_SD_CD);
  }
  if (!mountSdmmc()) {
    Serial.println("[SD_MMC] Mount failed; UI still available for diagnostics");
  } else {
    uint8_t type = SD_MMC.cardType();
    Serial.printf("[SD_MMC] Type=%u  Size=%llu MB\n",
                  (unsigned)type, (unsigned long long)(SD_MMC.cardSize() / (1024ULL*1024ULL)));
  }

  // LEDs
  for (int a = 0; a < NUM_ARMS; ++a) {
    strips[a] = new Adafruit_DotStar(LED_COUNT, ARM_DATA[a], ARM_CLK[a], DOTSTAR_BGR);
    strips[a]->begin();
    strips[a]->setBrightness(g_brightness);
    strips[a]->show();
  }
  blackoutAll();

  g_bootMs   = millis();
  g_playing  = false; // wait for user; timeout will auto-play test2
  g_currentPath = "";
  Serial.println("[STATE] Waiting for selection via web UI (5-min timeout to /test2.fseq)");
}

// ---------------- main loop ----------------
void loop() {
  server.handleClient();

  // Auto-timeout: if not playing within 5 min, start /test2.fseq
  if (!g_playing && (millis() - g_bootMs > SELECT_TIMEOUT_MS)) {
    Serial.println("[TIMEOUT] No selection made; auto-starting /test2.fseq");
    openFseq("/test2.fseq");
  }

  // Serial control shortcuts
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (!cmd.length()) { /* no-op */ }
    else if (cmd[0] == 'B' || cmd[0] == 'b') {
      int pct = cmd.substring(1).toInt();
      if (pct < 0) pct = 0; if (pct > 100) pct = 100;
      applyBrightness((uint8_t)pct);
    } else if (cmd.equalsIgnoreCase("STOP")) {
      stopPlayback();
    } else if (cmd.equalsIgnoreCase("PLAY")) {
      if (!g_currentPath.length()) g_currentPath = "/test2.fseq";
      openFseq(g_currentPath);
    } else if (cmd.equalsIgnoreCase("PAUSE")) {
      g_paused = true;  Serial.println("[DBG] paused");
    } else if (cmd.equalsIgnoreCase("RESUME")) {
      g_paused = false; g_lastTickMs = millis(); Serial.println("[DBG] resumed");
    } else if (cmd.startsWith("STEP")) {
      // STEP or STEP -1
      int dir = 1; int tmp;
      if (sscanf(cmd.c_str(), "STEP %d", &tmp) == 1) dir = tmp;
      // mimic /step behavior
      g_paused = true;
      if (dir >= 0) { renderFrame(g_frameIndex); g_frameIndex = (g_frameIndex + 1) % frames; }
      else { uint32_t prev = (g_frameIndex + frames - 1) % frames; renderFrame(prev); }
      Serial.printf("[DBG] step dir=%d next=%lu\n", dir, (unsigned long)g_frameIndex);
    } else if (cmd.startsWith("SPEED")) {
      int val = 0; if (sscanf(cmd.c_str(), "SPEED %d", &val) == 1) {
        if (val < 1) val = 1; if (val > 120) val = 120;
        g_fps = (uint16_t)val; g_framePeriodMs = 1000UL / g_fps; prefs.putUShort("fps", g_fps);
        g_lastTickMs = millis();
        Serial.printf("[DBG] FPS=%u period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
      } else {
        Serial.println("Usage: SPEED <fps 1..120>");
      }
    } else if (cmd.startsWith("REV")) {
      int a=-1,on=-1; sscanf(cmd.c_str(), "REV %d %d", &a, &on);
      if (a>=0 && a<NUM_ARMS && (on==0 || on==1)) {
        g_armReverse[a]=(on==1); Serial.printf("[DBG] arm %d reverse=%d\n", a,on);
      }
    } else if (cmd.startsWith("ORDER")) {
      char m[4]={0}; sscanf(cmd.c_str(), "ORDER %3s", m);
      String M = String(m); M.toUpperCase();
      if      (M=="RGB") g_colorMap=MAP_RGB;
      else if (M=="RBG") g_colorMap=MAP_RBG;
      else if (M=="GBR") g_colorMap=MAP_GBR;
      else if (M=="GRB") g_colorMap=MAP_GRB;
      else if (M=="BRG") g_colorMap=MAP_BRG;
      else if (M=="BGR") g_colorMap=MAP_BGR;
      Serial.printf("[DBG] color order=%s\n", M.c_str());
    } else if (cmd.startsWith("MAP")) {
      // MAP <arm> <split> <ra> <rb>
      int a=-1,s=-1,ra=-1,rb=-1;
      if (sscanf(cmd.c_str(), "MAP %d %d %d %d", &a, &s, &ra, &rb)==4) {
        if (a>=0 && a<NUM_ARMS) {
          if (s<0) s=0; if (s>(int)LED_COUNT) s=LED_COUNT;
          g_armSplit[a]=(uint16_t)s; g_segRevA[a]=(ra!=0); g_segRevB[a]=(rb!=0);
          Serial.printf("[MAP] arm=%d split=%d ra=%d rb=%d\n", a, s, ra, rb);
        }
      } else {
        Serial.println("Usage: MAP <arm> <split> <ra> <rb>");
      }
    }
  }

  // Scheduler tick (auto-advance if playing and not paused)
  tickPlayer();
}
