#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <stdlib.h>
#include <string.h>
#include "QuadMap.h"
#include "WebPages.h"

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
static SemaphoreHandle_t g_sdMutex = nullptr;
static inline bool SD_LOCK(TickType_t to = portMAX_DELAY) {
  return g_sdMutex && xSemaphoreTake(g_sdMutex, to) == pdTRUE;
}
static inline void SD_UNLOCK() { if (g_sdMutex) xSemaphoreGive(g_sdMutex); }


// ---------- SK9822 / APA102 ----------
static const uint8_t  MAX_ARMS               = 4;
static const uint16_t DEFAULT_PIXELS_PER_ARM = 144;
static const uint16_t MAX_PIXELS_PER_ARM     = 1024;

// Latest arm pin map (CLK and DATA per arm)
static const int ARM_CLK[MAX_ARMS]  = { 48, 35, 38,  2 };
static const int ARM_DATA[MAX_ARMS] = { 45, 36, 39, 42 };


// ---------- SD-MMC pins (your new map) ----------
static const int PIN_SD_CLK = 10;
static const int PIN_SD_CMD = 9;
static const int PIN_SD_D0  = 8;
// keep defined for later 4-bit
static const int PIN_SD_D1  = 13;
static const int PIN_SD_D2  = 12;
static const int PIN_SD_D3  = 11;
static const int PIN_SD_CD  = 14;  // LOW = inserted

// SD dynamic timing & fail tracking
static uint32_t g_sdFreqKHz   = 8000;  // start conservative; raise after stable
static int      g_sdFailStreak = 0;    // consecutive read failures

// Persistent scratch for zlib frames
static uint8_t* s_ctmp = nullptr;
static size_t   s_ctmp_size = 0;

// === Quadrant mapping controls ===
static int START_SPOKE_1BASED = 1;
static SpokeLabelMode gLabelMode = FLOOR_TO_BOUNDARY; // used for logging
static const int SPOKES = 40;

// ---------- Wi-Fi AP / STA ----------
static const char* AP_SSID  = "POV-Spinner";
static const char* AP_PASS  = "POV123456";
static const IPAddress AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);
static const char* DEFAULT_STATION_ID = "pov-spinner";
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t WIFI_RETRY_DELAY_MS     = 5000;

String     g_wifiSsid;
String     g_wifiPass;
String     g_stationId = DEFAULT_STATION_ID;
bool       g_stationConnected = false;
IPAddress  g_stationIp;
bool       g_stationConnectPending = false;
bool       g_stationReconnectScheduled = false;
uint32_t   g_stationConnectStartMs = 0;
uint32_t   g_stationReconnectAtMs  = 0;

WebServer server(80);

// ---------- Persisted settings (NVS = flash) ----------
Preferences prefs;
uint8_t  g_brightnessPercent = 25;
uint8_t  g_brightness        = 63;
uint16_t g_fps               = 40;
uint32_t g_framePeriodMs     = 25;

struct PrefMissingFlags {
  bool brightness = false;
  bool fps = false;
  bool startCh = false;
  bool spokes = false;
  bool arms = false;
  bool pixels = false;
  bool stride = false;
  bool wifiSsid = false;
  bool wifiPass = false;
  bool stationId = false;
};

PrefMissingFlags g_missing;

// Spinner model mapping (persisted)
uint32_t g_startChArm1   = 1;    // 1-based absolute channel (R of Arm1, Pixel0)
uint16_t g_spokesTotal   = 40;
uint8_t  g_armCount      = MAX_ARMS;
uint16_t g_pixelsPerArm  = DEFAULT_PIXELS_PER_ARM;

enum StrideMode : uint8_t { STRIDE_SPOKE=0, STRIDE_LED=1 };
StrideMode g_strideMode = STRIDE_SPOKE;

// Rotary index (0..spokes-1)
volatile uint16_t g_indexPosition = 0;

// Playback state
volatile bool  g_playing = false, g_paused = false;
String         g_currentPath;
uint32_t       g_frameIndex = 0, g_lastTickMs = 0;
uint32_t       g_bootMs = 0;
const uint32_t SELECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;

Adafruit_DotStar* strips[MAX_ARMS] = { nullptr };

/* -------------------- Prototypes for all web handlers -------------------- */
static void handleFiles();
static void handleDownload();
static void handlePlayLink();
static void handleDelete();
static void handleMkdir();
static void handleRename();
static void handleUploadData();
static void handleUploadDone();
static void handleStatus();
static void handleRoot();
static void handleB();
static void handleStart();
static void handleStop();
static void handleSpeed();
static void handleMapCfg();
static void handleFseqHeader();
static void handleCBlocks();
static void handleSdReinit();
static void handleWifiConfig();
static void handleOtaApply();

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
static inline uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static inline int32_t  clampI32(int32_t v, int32_t lo, int32_t hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static inline uint8_t  clampArmCount(int32_t v){ if(v<1) return 1; if(v>MAX_ARMS) return MAX_ARMS; return (uint8_t)v; }
static inline uint16_t clampPixelsPerArm(int32_t v){ if(v<1) return 1; if(v>MAX_PIXELS_PER_ARM) return MAX_PIXELS_PER_ARM; return (uint16_t)v; }
static inline uint8_t  activeArmCount(){ return (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount); }

static bool isFseqName(const String& n) {
  int dot = n.lastIndexOf('.');
  if (dot < 0) return false;
  String ext = n.substring(dot + 1); ext.toLowerCase();
  return ext == "fseq";
}

static String htmlEscape(const String& in) {
  String s; s.reserve(in.length()+8);
  for (size_t i=0;i<in.length();++i){
    char c=in[i];
    if(c=='&') s+="&amp;";
    else if(c=='<') s+="&lt;";
    else if(c=='>') s+="&gt;";
    else if(c=='\"') s+="&quot;";
    else if(c=='\'') s+="&#39;";
    else s+=c;
  }
  return s;
}
static String urlEncode(const String& in){
  const char* hex="0123456789ABCDEF";
  String s; s.reserve(in.length()*3);
  for (size_t i=0;i<in.length();++i){
    uint8_t c=(uint8_t)in[i];
    if( (c>='A'&&c<='Z') || (c>='a'&&c<='z') || (c>='0'&&c<='9') || c=='-'||c=='_'||c=='.'||c=='/' ){
      s+=(char)c;
    } else {
      s+='%'; s+=hex[c>>4]; s+=hex[c&0xF];
    }
  }
  return s;
}
static int fromHex(char c){ if(c>='0'&&c<='9') return c-'0'; if(c>='A'&&c<='F') return 10+c-'A'; if(c>='a'&&c<='f') return 10+c-'a'; return -1; }
static String urlDecode(const String& in){
  String s; s.reserve(in.length());
  for (size_t i=0;i<in.length();++i){
    char c=in[i];
    if(c=='%'){
      if(i+2<in.length()){
        int hi=fromHex(in[i+1]);
        int lo=fromHex(in[i+2]);
        if(hi>=0&&lo>=0){ s+=(char)((hi<<4)|lo); i+=2; continue; }
      }
      s+=c;
    } else if(c=='+'){
      s+=' ';
    } else {
      s+=c;
    }
  }
  return s;
}
static String jsonEscape(const String& in){
  String out; out.reserve(in.length()+8);
  for(size_t i=0;i<in.length();++i){
    char c=in[i];
    if(c=='"') out+="\\\"";
    else if(c=='\\') out+="\\\\";
    else if((uint8_t)c<0x20){
      char buf[7];
      snprintf(buf,sizeof(buf),"\\u%04X", (unsigned)(uint8_t)c);
      out+=buf;
    } else {
      out+=c;
    }
  }
  return out;
}
static String dirnameOf(const String& path){
  if(path=="/") return "/";
  int slash = path.lastIndexOf('/');
  if (slash<=0) return "/";
  return path.substring(0, slash);
}
static String joinPath(const String& dir, const String& name){
  String d = dir;
  if (!d.length() || d[0] != '/') d = "/" + d;
  if (d != "/" && d.endsWith("/")) d.remove(d.length()-1);
  String base = name;
  int slash = base.lastIndexOf('/');
  if (slash >= 0) base = base.substring(slash+1);
  return (d == "/") ? ("/" + base) : (d + "/" + base);
}

/* -------------------- Settings backup helpers -------------------- */
static const char* SETTINGS_DIR        = "/config";
static const char* SETTINGS_PATH       = "/config/settings.cfg";
static const char* SETTINGS_TMP_PATH   = "/config/settings.tmp";

struct SettingsBackupData {
  bool hasBrightness = false; uint8_t brightness = 25;
  bool hasFps = false;        uint16_t fps = 40;
  bool hasStartCh = false;    uint32_t startCh = 1;
  bool hasSpokes = false;     uint16_t spokes = 40;
  bool hasArms = false;       uint8_t arms = MAX_ARMS;
  bool hasPixels = false;     uint16_t pixels = DEFAULT_PIXELS_PER_ARM;
  bool hasStride = false;     uint8_t stride = (uint8_t)STRIDE_SPOKE;
  bool hasWifiSsid = false;   String wifiSsid;
  bool hasWifiPass = false;   String wifiPass;
  bool hasStationId = false;  String stationId;
};

static SettingsBackupData currentSettingsForBackup(){
  SettingsBackupData data;
  data.hasBrightness = true; data.brightness = g_brightnessPercent;
  data.hasFps = true;        data.fps = g_fps;
  data.hasStartCh = true;    data.startCh = g_startChArm1;
  data.hasSpokes = true;     data.spokes = g_spokesTotal;
  data.hasArms = true;       data.arms = g_armCount;
  data.hasPixels = true;     data.pixels = g_pixelsPerArm;
  data.hasStride = true;     data.stride = (uint8_t)g_strideMode;
  data.hasWifiSsid = true;   data.wifiSsid = g_wifiSsid;
  data.hasWifiPass = true;   data.wifiPass = g_wifiPass;
  data.hasStationId = true;  data.stationId = g_stationId;
  return data;
}

static bool ensureSettingsDirLocked(){
  if (!SD_MMC.exists(SETTINGS_DIR)) {
    if (!SD_MMC.mkdir(SETTINGS_DIR)) {
      Serial.printf("[SETTINGS] mkdir %s failed\n", SETTINGS_DIR);
      return false;
    }
  }
  return true;
}

static bool saveSettingsBackup(){
  if (!g_sdMutex) return false;
  if (!cardPresent()) return false;
  SettingsBackupData data = currentSettingsForBackup();
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return false;
  bool ok = false;
  do {
    if (!ensureSettingsDirLocked()) break;
    if (SD_MMC.exists(SETTINGS_TMP_PATH)) SD_MMC.remove(SETTINGS_TMP_PATH);
    File f = SD_MMC.open(SETTINGS_TMP_PATH, FILE_WRITE);
    if (!f) { Serial.println("[SETTINGS] open tmp fail"); break; }
    f.print("version=1\n");
    f.printf("brightness=%u\n", (unsigned)data.brightness);
    f.printf("fps=%u\n", (unsigned)data.fps);
    f.printf("startCh=%lu\n", (unsigned long)data.startCh);
    f.printf("spokes=%u\n", (unsigned)data.spokes);
    f.printf("arms=%u\n", (unsigned)data.arms);
    f.printf("pixels=%u\n", (unsigned)data.pixels);
    f.printf("stride=%u\n", (unsigned)data.stride);
    f.print("wifiSsid=" + urlEncode(data.wifiSsid) + "\n");
    f.print("wifiPass=" + urlEncode(data.wifiPass) + "\n");
    f.print("stationId=" + urlEncode(data.stationId) + "\n");
    f.close();
    if (SD_MMC.exists(SETTINGS_PATH)) SD_MMC.remove(SETTINGS_PATH);
    if (!SD_MMC.rename(SETTINGS_TMP_PATH, SETTINGS_PATH)) {
      Serial.println("[SETTINGS] rename tmp->cfg fail");
      break;
    }
    ok = true;
  } while(false);
  if (!ok && SD_MMC.exists(SETTINGS_TMP_PATH)) SD_MMC.remove(SETTINGS_TMP_PATH);
  SD_UNLOCK();
  return ok;
}

static bool loadSettingsBackup(SettingsBackupData& out){
  if (!g_sdMutex) return false;
  if (!cardPresent()) return false;
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return false;
  bool ok = false;
  do {
    if (!SD_MMC.exists(SETTINGS_PATH)) break;
    File f = SD_MMC.open(SETTINGS_PATH, FILE_READ);
    if (!f) { Serial.println("[SETTINGS] open cfg fail"); break; }
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length() || line[0]=='#') continue;
      int eq = line.indexOf('=');
      if (eq < 0) continue;
      String key = line.substring(0, eq);
      String val = line.substring(eq+1);
      if (key == "brightness") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 0) v = 0; if (v > 100) v = 100;
        out.brightness = (uint8_t)v; out.hasBrightness = true;
      } else if (key == "fps") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 1) v = 1; if (v > 120) v = 120;
        out.fps = (uint16_t)v; out.hasFps = true;
      } else if (key == "startCh") {
        unsigned long v = strtoul(val.c_str(), nullptr, 10);
        if (v < 1) v = 1;
        out.startCh = v; out.hasStartCh = true;
      } else if (key == "spokes") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 1) v = 1;
        out.spokes = (uint16_t)v; out.hasSpokes = true;
      } else if (key == "arms") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 1) v = 1; if (v > MAX_ARMS) v = MAX_ARMS;
        out.arms = (uint8_t)v; out.hasArms = true;
      } else if (key == "pixels") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 1) v = 1; if (v > MAX_PIXELS_PER_ARM) v = MAX_PIXELS_PER_ARM;
        out.pixels = (uint16_t)v; out.hasPixels = true;
      } else if (key == "stride") {
        long v = strtol(val.c_str(), nullptr, 10);
        if (v < 0) v = 0; if (v > 1) v = 1;
        out.stride = (uint8_t)v; out.hasStride = true;
      } else if (key == "wifiSsid") {
        out.wifiSsid = urlDecode(val); out.hasWifiSsid = true;
      } else if (key == "wifiPass") {
        out.wifiPass = urlDecode(val); out.hasWifiPass = true;
      } else if (key == "stationId") {
        out.stationId = urlDecode(val); out.hasStationId = true;
      }
    }
    f.close();
    ok = true;
  } while(false);
  SD_UNLOCK();
  return ok;
}

static void computeDerivedSettings(){
  g_brightness = (uint8_t)((255 * g_brightnessPercent) / 100);
  if (!g_fps) g_fps = 1;
  g_framePeriodMs = 1000UL / g_fps;
}

static bool anyMissingPrefs(){
  return g_missing.brightness || g_missing.fps || g_missing.startCh ||
         g_missing.spokes || g_missing.arms || g_missing.pixels ||
         g_missing.stride || g_missing.wifiSsid || g_missing.wifiPass ||
         g_missing.stationId;
}

static void restoreSettingsFromBackupIfNeeded(){
  if (!anyMissingPrefs()) return;
  SettingsBackupData data;
  if (!loadSettingsBackup(data)) return;
  bool updated = false;
  if (g_missing.brightness && data.hasBrightness) {
    g_brightnessPercent = data.brightness;
    prefs.putUChar("brightness", g_brightnessPercent);
    g_missing.brightness = false;
    updated = true;
  }
  if (g_missing.fps && data.hasFps) {
    g_fps = data.fps;
    prefs.putUShort("fps", g_fps);
    g_missing.fps = false;
    updated = true;
  }
  if (g_missing.startCh && data.hasStartCh) {
    g_startChArm1 = data.startCh;
    prefs.putULong("startch", g_startChArm1);
    g_missing.startCh = false;
    updated = true;
  }
  if (g_missing.spokes && data.hasSpokes) {
    g_spokesTotal = data.spokes;
    prefs.putUShort("spokes", g_spokesTotal);
    g_missing.spokes = false;
    updated = true;
  }
  if (g_missing.arms && data.hasArms) {
    g_armCount = data.arms;
    prefs.putUChar("arms", g_armCount);
    g_missing.arms = false;
    updated = true;
  }
  if (g_missing.pixels && data.hasPixels) {
    g_pixelsPerArm = data.pixels;
    prefs.putUShort("pixels", g_pixelsPerArm);
    g_missing.pixels = false;
    updated = true;
  }
  if (g_missing.stride && data.hasStride) {
    g_strideMode = (StrideMode)data.stride;
    prefs.putUChar("stride", (uint8_t)g_strideMode);
    g_missing.stride = false;
    updated = true;
  }
  if (g_missing.wifiSsid && data.hasWifiSsid) {
    g_wifiSsid = data.wifiSsid;
    prefs.putString("wssid", g_wifiSsid);
    g_missing.wifiSsid = false;
    updated = true;
  }
  if (g_missing.wifiPass && data.hasWifiPass) {
    g_wifiPass = data.wifiPass;
    prefs.putString("wpass", g_wifiPass);
    g_missing.wifiPass = false;
    updated = true;
  }
  if (g_missing.stationId && data.hasStationId) {
    g_stationId = data.stationId.length() ? data.stationId : DEFAULT_STATION_ID;
    prefs.putString("wsid", g_stationId);
    g_missing.stationId = false;
    updated = true;
  }
  if (updated) {
    computeDerivedSettings();
    saveSettingsBackup();
  }
}

static void updateHostname(){
  String host = g_stationId.length() ? g_stationId : String(DEFAULT_STATION_ID);
  g_stationId = host;
  WiFi.setHostname(host.c_str());
}

static void startStationConnection(){
  if (!g_wifiSsid.length()) {
    WiFi.disconnect(false);
    g_stationConnected = false;
    g_stationConnectPending = false;
    return;
  }
  Serial.printf("[WIFI] Connecting to STA SSID '%s'\n", g_wifiSsid.c_str());
  updateHostname();
  WiFi.begin(g_wifiSsid.c_str(), g_wifiPass.c_str());
  g_stationConnectPending = true;
  g_stationConnectStartMs = millis();
}

static void scheduleStationReconnect(uint32_t delayMs){
  g_stationReconnectScheduled = true;
  g_stationReconnectAtMs = millis() + delayMs;
  g_stationConnectPending = false;
}

static void manageWifiStation(){
  if (!g_wifiSsid.length()) {
    g_stationConnected = false;
    g_stationConnectPending = false;
    g_stationReconnectScheduled = false;
    g_stationIp = IPAddress();
    return;
  }

  wl_status_t st = WiFi.status();
  if (g_stationConnectPending) {
    if (st == WL_CONNECTED) {
      g_stationConnected = true;
      g_stationConnectPending = false;
      g_stationIp = WiFi.localIP();
      Serial.printf("[WIFI] STA connected, IP=%s\n", g_stationIp.toString().c_str());
    } else if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
      Serial.println("[WIFI] STA connect failed");
      WiFi.disconnect(false);
      g_stationConnectPending = false;
      g_stationConnected = false;
      g_stationIp = IPAddress();
      scheduleStationReconnect(WIFI_RETRY_DELAY_MS);
    } else if (millis() - g_stationConnectStartMs > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("[WIFI] STA connect timeout");
      WiFi.disconnect(false);
      g_stationConnectPending = false;
      g_stationConnected = false;
      g_stationIp = IPAddress();
      scheduleStationReconnect(WIFI_RETRY_DELAY_MS);
    }
  } else {
    if (g_stationConnected) {
      if (st != WL_CONNECTED) {
        Serial.println("[WIFI] STA lost connection");
        g_stationConnected = false;
        g_stationIp = IPAddress();
        scheduleStationReconnect(WIFI_RETRY_DELAY_MS);
      }
    } else if (g_stationReconnectScheduled && millis() >= g_stationReconnectAtMs) {
      g_stationReconnectScheduled = false;
      startStationConnection();
    }
  }
}

/* -------------------- SD helpers (strict 1-bit + mutex) -------------------- */
static bool cardPresent(){ pinMode(PIN_SD_CD, INPUT_PULLUP); return digitalRead(PIN_SD_CD)==LOW; }
static void sd_preflight() {
  pinMode(PIN_SD_CMD, INPUT_PULLUP);
  pinMode(PIN_SD_D0,  INPUT_PULLUP);
  delay(2);
  Serial.printf("[SD] Preflight CMD@%d=%d  D0@%d=%d (expect 1,1)\n",
                PIN_SD_CMD, digitalRead(PIN_SD_CMD),
                PIN_SD_D0,  digitalRead(PIN_SD_D0));
}
static bool mountSdmmc(){
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { Serial.println("[SD] mount lock timeout"); return false; }
  bool ok=false;
  do {
    sd_preflight();
    // STRICT 1-bit: do not pass D1..D3 here
    SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, -1, -1, -1);
    ok = SD_MMC.begin("/sdcard", true /*1-bit*/, false /*no-format*/, g_sdFreqKHz);
    Serial.printf("[SD] Mounted (1b) @ %lu kHz: %s\n", (unsigned long)g_sdFreqKHz, ok?"OK":"FAIL");
  } while(0);
  SD_UNLOCK();
  return ok;
}

/* -------------------- Endianness readers -------------------- */
static bool readU16(File& f, uint16_t& v){ uint8_t b[2]; if(f.read(b,2)!=2) return false; v=(uint16_t)(b[0]|(b[1]<<8)); return true; }
static bool readU32(File& f, uint32_t& v){ uint8_t b[4]; if(f.read(b,4)!=4) return false; v=(uint32_t)(b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24)); return true; }
static bool readU64(File& f, uint64_t& v){ uint8_t b[8]; if(f.read(b,8)!=8) return false;
  v=(uint64_t)b[0]|((uint64_t)b[1]<<8)|((uint64_t)b[2]<<16)|((uint64_t)b[3]<<24)|
    ((uint64_t)b[4]<<32)|((uint64_t)b[5]<<40)|((uint64_t)b[6]<<48)|((uint64_t)b[7]<<56);
  return true;
}

/* -------------------- zlib inflate wrapper -------------------- */
static bool zlib_decompress(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_len){
#if defined(MZ_OK)
  mz_ulong dst = (mz_ulong)out_len;
  int rc = mz_uncompress((unsigned char*)out, &dst,
                         (const unsigned char*)in, (mz_ulong)in_len);
  return (rc == MZ_OK) && (dst == (mz_ulong)out_len);
#elif defined(Z_OK)
  uLongf dst = (uLongf)out_len;
  int rc = uncompress((Bytef*)out, &dst, (const Bytef*)in, (uLong)in_len);
  return (rc == Z_OK) && (dst == (uLongf)out_len);
#else
  (void)in; (void)in_len; (void)out; (void)out_len; return false;
#endif
}

/* -------------------- FSEQ open/close/load -------------------- */
static void freeFseq(){
  if (g_fseq) g_fseq.close();
  if (g_ranges){ free(g_ranges); g_ranges=nullptr; }
  if (g_frameBuf){ free(g_frameBuf); g_frameBuf=nullptr; }
  if (g_cblocks){ free(g_cblocks); g_cblocks=nullptr; }
  if (s_ctmp){ free(s_ctmp); s_ctmp=nullptr; s_ctmp_size=0; }
  g_compCount=0; g_compBase=0; g_compPerFrame=false;
  memset(&g_fh,0,sizeof(g_fh));
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

static bool openFseq(const String& path, String& why){
  freeFseq();
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { why="sd busy"; return false; }
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

    g_currentPath = path; g_frameIndex=0; g_lastTickMs = millis(); g_playing = true;
    Serial.printf("[FSEQ] %s frames=%lu chans=%lu step=%ums comp=%u blocks=%u sparse=%u CDO=0x%04x\n",
      path.c_str(), (unsigned long)g_fh.frameCount, (unsigned long)g_fh.channelCount,
      g_fh.stepTimeMs, g_fh.compType, (unsigned)g_compCount, (unsigned)g_fh.sparseCnt, g_fh.chanDataOffset);

    ok = true;
  } while(0);
  SD_UNLOCK();

  if (!ok) { freeFseq(); }
  return ok;
}

static bool loadFrame(uint32_t idx){
  if (!g_fseq || !g_fh.frameCount) return false;
  idx %= g_fh.frameCount;

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return false;
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

static void rebuildStrips(){
  const uint8_t arms = activeArmCount();
  const uint16_t pixels = g_pixelsPerArm ? g_pixelsPerArm : 1;

  for (uint8_t a=0; a<MAX_ARMS; ++a){
    if (strips[a]){ delete strips[a]; strips[a] = nullptr; }
  }
  for (uint8_t a=0; a<arms; ++a){
    strips[a] = new Adafruit_DotStar(pixels, ARM_DATA[a], ARM_CLK[a], DOTSTAR_BGR);
    strips[a]->begin();
    strips[a]->setBrightness(g_brightness);
    strips[a]->show();
  }
}

static void blackoutAll(){
  const uint8_t arms = activeArmCount();
  for (uint8_t a=0; a<arms; ++a){
    if (!strips[a]) continue;
    uint16_t pix = strips[a]->numPixels();
    for (uint16_t i=0; i<pix; ++i) strips[a]->setPixelColor(i,0,0,0);
    strips[a]->show();
  }
}

static bool renderFrame(uint32_t idx){
  if (!loadFrame(idx)) return false;

  const uint32_t spokes = (g_spokesTotal ? g_spokesTotal : 1);
  const uint8_t  arms = activeArmCount();
  const uint32_t blockStride = (g_strideMode==STRIDE_SPOKE && spokes) ? (spokes*3UL) : 3UL;

  const uint32_t fallbackPixels = g_pixelsPerArm ? g_pixelsPerArm : DEFAULT_PIXELS_PER_ARM;
  const uint32_t chPerSpoke =
    (spokes > 0 && g_fh.channelCount) ? (g_fh.channelCount / spokes) : (fallbackPixels * 3u);

  const uint32_t startChBase = (g_startChArm1 > 0 ? g_startChArm1 - 1 : 0);
  const uint32_t indexOffset = (spokes > 0) ? (g_indexPosition % spokes) : 0;

  const int startIdx0 = spoke1BasedToIdx0(START_SPOKE_1BASED, (int)spokes);

  for (uint8_t arm=0; arm<arms; ++arm){
    if (!strips[arm]) continue;

    uint32_t armBaseSpoke = (uint32_t)armSpokeIdx0((int)arm, startIdx0, (int)spokes, (int)arms);
    armBaseSpoke = (armBaseSpoke + indexOffset) % spokes;

    const uint32_t baseChAbsR = startChBase + armBaseSpoke * chPerSpoke;
    const uint32_t pixelCount = strips[arm]->numPixels();

    for (uint32_t i=0; i<pixelCount; ++i){
      const uint32_t absR = baseChAbsR + i * blockStride;
      const int64_t  idxR = sparseTranslate(absR);
      if (idxR < 0 || (idxR+2) >= (int64_t)g_fh.channelCount) { strips[arm]->setPixelColor(i,0,0,0); continue; }
      uint8_t R,G,B; mapChannels(&g_frameBuf[idxR], R,G,B);
      strips[arm]->setPixelColor(i, R,G,B);
    }
    strips[arm]->show();
  }
  return true;
}

/* -------------------- Web: Files page + ops -------------------- */
static void listFseqInDir_locked(const char* path, String& optionsHtml, uint8_t depth = 0) {
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
  File ent;
  while ((ent = dir.openNextFile())) {
    String name = ent.name();
    if (ent.isDirectory()) {
      if (depth == 0) listFseqInDir_locked(name.c_str(), optionsHtml, depth + 1);
    } else if (isFseqName(name)) {
      optionsHtml += "<option value='"; optionsHtml += name; optionsHtml += "'>";
      optionsHtml += name; optionsHtml += "</option>";
    }
    ent.close();
  }
  dir.close();
}
static void listFseqInDir(const char* path, String& optionsHtml, uint8_t depth = 0) {
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { Serial.println("[SD] busy; skip list"); return; }
  listFseqInDir_locked(path, optionsHtml, depth);
  SD_UNLOCK();
}

static void handleFiles() {
  String path = server.hasArg("path") ? server.arg("path") : "/";
  if (!path.length() || path[0] != '/') path = "/" + path;

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close(); SD_UNLOCK();
    server.send(404, "text/plain", "Directory not found");
    return;
  }

  String parent = dirnameOf(path);
  String pathEsc = htmlEscape(path);
  String parentEnc = urlEncode(parent);
  String pathEnc = urlEncode(path);
  String backPlain = String("/files?path=") + path;
  String backParam = String("/files?path=") + pathEnc;
  String backEncoded = urlEncode(backPlain);

  String html = WebPages::filesPageHeader(pathEsc, parentEnc, pathEnc, backEncoded);

  File ent;
  while ((ent = dir.openNextFile())) {
    String name = ent.name();
    String esc = htmlEscape(name);
    String enc = urlEncode(name);
    if (ent.isDirectory()) {
      String base = name; int slash = base.lastIndexOf('/'); if (slash >= 0) base = base.substring(slash + 1);
      String baseEsc = htmlEscape(base);
      html += WebPages::filesDirectoryRow(esc, enc, esc, baseEsc, backParam);
    } else {
      uint64_t sz = ent.size();
      String base = name; int slash = base.lastIndexOf('/'); if (slash >= 0) base = base.substring(slash + 1);
      String baseEsc = htmlEscape(base);
      html += WebPages::filesFileRow(esc, enc, sz, esc, baseEsc, backParam);
    }
    ent.close();
  }
  dir.close();
  SD_UNLOCK();

  html += WebPages::filesPageFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleDownload() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path"); if (!path.startsWith("/")) path = "/" + path;

  if (!SD_LOCK(pdMS_TO_TICKS(5000))) { server.send(503,"text/plain","SD busy"); return; }
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) { if (f) f.close(); SD_UNLOCK(); server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + htmlEscape(path.substring(path.lastIndexOf('/')+1)) + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
  SD_UNLOCK();
}

static void handlePlayLink() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.exists(path) && isFseqName(path);
  SD_UNLOCK();
  if (!ok) { server.sendHeader("Location", back); server.send(302, "text/plain", "Not a .fseq or missing"); return; }

  String why; openFseq(path, why);
  server.sendHeader("Location", back);
  server.send(302, "text/plain", "OK");
}

static void handleDelete() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
  File f = SD_MMC.open(path);
  bool ok=false;
  if (f) {
    if (f.isDirectory()) { f.close(); ok = SD_MMC.rmdir(path); }
    else { f.close(); ok = SD_MMC.remove(path); }
  }
  SD_UNLOCK();
  server.sendHeader("Location", back);
  server.send(ok?302:500, "text/plain", ok?"Deleted":"Delete failed");
}

static void handleMkdir() {
  if (!server.hasArg("path") || !server.hasArg("name")) { server.send(400, "text/plain", "args"); return; }
  String base = server.arg("path");
  String name = server.arg("name");
  if (!base.startsWith("/")) base = "/" + base;
  if (name.indexOf('/')>=0 || !name.length()) { server.send(400, "text/plain", "bad name"); return; }
  if (!base.endsWith("/")) base += "/";

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.mkdir(base + name);
  SD_UNLOCK();

  server.sendHeader("Location", String("/files?path=") + urlEncode(base.substring(0, base.length()-1)));
  server.send(ok?302:500, "text/plain", ok?"Created":"Create failed");
}

static void handleRename() {
  if (!server.hasArg("path") || !server.hasArg("to")) { server.send(400, "text/plain", "args"); return; }
  String p = server.arg("path");
  String to = server.arg("to");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!p.startsWith("/")) p = "/" + p;
  if (to.indexOf('/')>=0 || !to.length()) { server.sendHeader("Location", back); server.send(302, "text/plain", "bad name"); return; }

  String dir = dirnameOf(p);
  String dst = (dir == "/") ? ("/" + to) : (dir + "/" + to);

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.rename(p, dst);
  SD_UNLOCK();

  server.sendHeader("Location", back);
  server.send(ok?302:500, "text/plain", ok?"Renamed":"Rename failed");
}

// Upload state
File   g_uploadFile;
String g_uploadFilename;
size_t g_uploadBytes = 0;

static void handleUploadData() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_uploadBytes = 0;

    String dir = server.hasArg("dir") ? server.arg("dir") : "/";
    if (dir.indexOf("..") >= 0) dir = "/";
    if (!dir.startsWith("/")) dir = "/" + dir;

    g_uploadFilename = up.filename;
    if (!g_uploadFilename.length()) g_uploadFilename = "upload.fseq";
    int slash = g_uploadFilename.lastIndexOf('/'); if (slash >= 0) g_uploadFilename = g_uploadFilename.substring(slash + 1);
    g_uploadFilename = joinPath(dir, g_uploadFilename);

    if (!isFseqName(g_uploadFilename)) {
      Serial.printf("[UPLOAD] Rejected non-.fseq: %s\n", g_uploadFilename.c_str());
    } else {
      if (!SD_LOCK(pdMS_TO_TICKS(5000))) { Serial.println("[UPLOAD] SD busy"); return; }
      File d = SD_MMC.open(dir);
      bool okdir = d && d.isDirectory(); if (d) d.close();
      if (!okdir) {
        Serial.printf("[UPLOAD] Target dir missing: %s\n", dir.c_str());
      } else {
        if (SD_MMC.exists(g_uploadFilename)) SD_MMC.remove(g_uploadFilename);
        g_uploadFile = SD_MMC.open(g_uploadFilename, FILE_WRITE);
        Serial.printf("[UPLOAD] START %s\n", g_uploadFilename.c_str());
      }
      SD_UNLOCK();
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_uploadFile) {
      if (SD_LOCK(pdMS_TO_TICKS(2000))) {
        g_uploadFile.write(up.buf, up.currentSize);
        SD_UNLOCK();
        g_uploadBytes += up.currentSize;
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_uploadFile) {
      if (SD_LOCK(pdMS_TO_TICKS(2000))) { g_uploadFile.close(); SD_UNLOCK(); }
      Serial.printf("[UPLOAD] DONE %s (%u bytes)\n", g_uploadFilename.c_str(), (unsigned)g_uploadBytes);
    } else {
      Serial.println("[UPLOAD] Aborted/invalid file");
    }
  }
}

static void handleUploadDone() {
  String back = server.hasArg("back") ? server.arg("back") : "/";
  bool ok=false;
  if (isFseqName(g_uploadFilename)) {
    if (SD_LOCK(pdMS_TO_TICKS(2000))) {
      ok = SD_MMC.exists(g_uploadFilename);
      SD_UNLOCK();
    }
  }
  if (!isFseqName(g_uploadFilename)) {
    server.send(415, "text/html",
      "<meta http-equiv='refresh' content='2;url=" + back + "'>"
      "<body style='font-family:system-ui'><p>Upload rejected. Only <b>.fseq</b> files are allowed.</p><p>Returning…</p></body>");
    return;
  }
  if (!ok) {
    server.send(500, "text/html",
      "<meta http-equiv='refresh' content='3;url=" + back + "'>"
      "<body style='font-family:system-ui'><p>Upload failed.</p><p>Returning…</p></body>");
    return;
  }
  server.send(200, "text/html",
    "<meta http-equiv='refresh' content='1;url=" + back + "'>"
    "<body style='font-family:system-ui'><p>Uploaded <b>" + htmlEscape(g_uploadFilename) + "</b> (" + String(g_uploadBytes) + " bytes).</p><p>Refreshing…</p></body>");
}

/* -------------------- Web: Control page & API -------------------- */
static inline const char* statusText() { if (g_paused && g_playing) return "Paused"; if (g_playing) return "Playing"; return "Stopped"; }
static inline const char* statusClass(){ if (g_paused && g_playing) return "badge pause"; if (g_playing) return "badge play"; return "badge stop"; }

static void handleStatus(){
  String staIpJson = g_stationConnected ? g_stationIp.toString() : String("");
  String json = String("{\"playing\":")+(g_playing?"true":"false")
    +",\"paused\":"+(g_paused?"true":"false")
    +",\"path\":\""+jsonEscape(g_currentPath)+"\""
    +",\"frame\":"+String(g_frameIndex)
    +",\"fps\":"+String(g_fps)
    +",\"startChArm1\":"+String(g_startChArm1)
    +",\"spokes\":"+String(g_spokesTotal)
    +",\"arms\":"+String(g_armCount)
    +",\"pixels\":"+String(g_pixelsPerArm)
    +",\"index\":"+String(g_indexPosition)
    +",\"stride\":\""+String(g_strideMode==STRIDE_SPOKE?"spoke":"led")+"\""
    +",\"wifiStaSsid\":\""+jsonEscape(g_wifiSsid)+"\""
    +",\"wifiStaConnected\":"+(g_stationConnected?"true":"false")
    +",\"wifiStaIp\":\""+ jsonEscape(staIpJson) +"\""
    +",\"stationId\":\""+jsonEscape(g_stationId)+"\""
    +"}";
  server.send(200,"application/json",json);
}

static void handleWifiConfig(){
  String ssid = server.hasArg("ssid") ? server.arg("ssid") : "";
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  String sid  = server.hasArg("id")   ? server.arg("id")   : "";

  ssid.trim();
  sid.trim();
  if (ssid.length() > 63) ssid = ssid.substring(0, 63);
  if (sid.length() > 31)  sid  = sid.substring(0, 31);

  g_wifiSsid = ssid;
  g_wifiPass = pass;
  g_stationId = sid.length() ? sid : String(DEFAULT_STATION_ID);

  prefs.putString("wssid", g_wifiSsid);
  prefs.putString("wpass", g_wifiPass);
  prefs.putString("wsid", g_stationId);
  saveSettingsBackup();

  g_stationConnected = false;
  g_stationIp = IPAddress();
  g_stationReconnectScheduled = false;
  g_stationConnectPending = false;

  updateHostname();
  MDNS.end();
  if (MDNS.begin(g_stationId.c_str())) MDNS.addService("http","tcp",80);

  if (g_wifiSsid.length()) {
    startStationConnection();
  } else {
    WiFi.disconnect(false);
  }

  server.send(200, "application/json", "{\"wifi\":\"saved\"}");
}

static void handleOtaApply(){
  if (!server.hasArg("path")) {
    server.send(400, "text/plain", "missing path");
    return;
  }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  if (!cardPresent()) {
    server.send(503, "text/plain", "sd missing");
    return;
  }

  String message;
  bool ok = false;
  File f;
  if (!SD_LOCK(pdMS_TO_TICKS(5000))) {
    server.send(503, "text/plain", "sd busy");
    return;
  }
  do {
    if (!SD_MMC.exists(path)) { message = "file not found"; break; }
    f = SD_MMC.open(path, FILE_READ);
    if (!f) { message = "open fail"; break; }
    size_t size = f.size();
    if (!size) { message = "empty file"; break; }
    Serial.printf("[OTA] Applying update from %s (%u bytes)\n", path.c_str(), (unsigned)size);
    if (!Update.begin(size)) {
      message = String("begin fail ") + Update.getError();
      break;
    }
    size_t written = Update.writeStream(f);
    if (written != size) {
      message = String("write mismatch ") + written;
      Update.abort();
      break;
    }
    if (!Update.end()) {
      message = String("end fail ") + Update.getError();
      break;
    }
    if (!Update.isFinished()) {
      message = "incomplete";
      break;
    }
    ok = true;
  } while(false);
  if (f) f.close();
  SD_UNLOCK();

  if (!ok) {
    Serial.printf("[OTA] Failed: %s\n", message.c_str());
    server.send(500, "text/plain", String("OTA failed: ") + message);
    return;
  }

  server.send(200, "application/json", "{\"ota\":\"rebooting\"}");
  Serial.println("[OTA] Success, rebooting");
  delay(250);
  ESP.restart();
}

static void handleRoot() {
  String options; listFseqInDir("/", options);
  String cur = g_currentPath.length() ? g_currentPath : "(none)";
  String curEsc = htmlEscape(cur);
  String apIp = AP_IP.toString();
  String staStatus = g_stationConnected ? "Connected" : "Not connected";
  String staIp = g_stationConnected ? g_stationIp.toString() : String("");
  String wifiSsidEsc = htmlEscape(g_wifiSsid);
  String wifiPassEsc = htmlEscape(g_wifiPass);
  String stationIdEsc = htmlEscape(g_stationId);
  String mdnsName = g_stationId.length() ? g_stationId + String(".local") : String("pov.local");
  String mdnsEsc = htmlEscape(mdnsName);
  String html = WebPages::rootPage(String(statusClass()), String(statusText()), curEsc, options,
                                   String(AP_SSID), apIp, mdnsEsc,
                                   wifiSsidEsc, staStatus, staIp, stationIdEsc, wifiPassEsc,
                                    g_startChArm1, g_spokesTotal, g_armCount, g_pixelsPerArm,
                                    MAX_ARMS, MAX_PIXELS_PER_ARM,
                                    g_strideMode == STRIDE_SPOKE, g_fps, g_brightnessPercent);

  server.send(200, "text/html; charset=utf-8", html);
}

static void applyBrightness(uint8_t pct){
  if (pct>100) pct=100;
  g_brightnessPercent=pct; g_brightness=(uint8_t)((255*pct)/100);
  const uint8_t arms = activeArmCount();
  for (uint8_t a=0; a<arms; ++a){ if (strips[a]){ strips[a]->setBrightness(g_brightness); strips[a]->show(); } }
  prefs.putUChar("brightness", g_brightnessPercent);
  saveSettingsBackup();
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

static void handleStart(){ // GET /start?path=/file.fseq
  if (server.hasArg("path")){
    String p = server.arg("path"); if (!p.startsWith("/")) p="/"+p;
    String why;
    if (!openFseq(p, why)){ server.send(500,"text/plain",String("FSEQ open failed: ")+why); return; }
  }
  g_playing=true; g_paused=false; g_lastTickMs=millis();
  server.send(200,"application/json","{\"playing\":true}");
}
static void handleStop(){ g_playing=false; blackoutAll(); server.send(200,"application/json","{\"playing\":false}"); }

static void handleSpeed() {
  if (!server.hasArg("fps")) { server.send(400, "text/plain", "missing fps"); return; }
  int val = server.arg("fps").toInt();
  if (val < 1) val = 1;
  if (val > 120) val = 120;
  g_fps = (uint16_t)val;
  g_framePeriodMs = (uint32_t) (1000UL / g_fps);
  prefs.putUShort("fps", g_fps);
  g_lastTickMs = millis();
  saveSettingsBackup();
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
  if (server.hasArg("stride")) {
    String s = server.arg("stride"); s.toLowerCase();
    g_strideMode = (s == "led") ? STRIDE_LED : STRIDE_SPOKE;
    prefs.putUChar("stride", (uint8_t)g_strideMode);
  }

  if (needRebuild) rebuildStrips();

  saveSettingsBackup();

  server.send(200, "application/json",
    String("{\"start\":") + g_startChArm1 +
    ",\"spokes\":" + g_spokesTotal +
    ",\"arms\":" + (int)g_armCount +
    ",\"pixels\":" + g_pixelsPerArm +
    ",\"stride\":\"" + (g_strideMode==STRIDE_SPOKE ? "spoke" : "led") + "\"}"
  );
}

// Diagnostics
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

static void handleSdReinit(){
  bool ok=false;
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  if (SD_MMC.cardType()!=CARD_NONE) ok=true;
  else ok = SD_MMC.begin("/sdcard", true, false, g_sdFreqKHz);
  SD_UNLOCK();

  if (!ok) { server.send(500,"text/plain","SD not present"); return; }
  if (!g_currentPath.length()) { server.send(200,"text/plain","SD OK; no file"); return; }
  String why;
  if (openFseq(g_currentPath, why)) server.send(200,"text/plain","SD OK; file reopened");
  else server.send(500,"text/plain", String("reopen fail: ")+why);
}

/* -------------------- SD Recovery Ladder -------------------- */
static bool recoverSd(const char* reason) {
  Serial.printf("[SD] Recover: %s  streak=%d  freq=%lu kHz  CD=%d\n",
      reason, g_sdFailStreak, (unsigned long)g_sdFreqKHz,
      (int)digitalRead(PIN_SD_CD));

  if (!cardPresent()) {
    Serial.println("[SD] Card not present (CD HIGH). Waiting...");
    uint32_t t0 = millis();
    while (!cardPresent() && millis() - t0 < 5000) { delay(50); server.handleClient(); }
    if (!cardPresent()) return false;
  }

  bool ok=false;

  if (g_sdFailStreak == 1) {
    if (g_currentPath.length()) {
      String why; ok = openFseq(g_currentPath, why);
      Serial.printf("[SD] Reopen file: %s\n", ok?"OK": why.c_str());
      if (ok) return true;
    }
  }

  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return false;
  SD_MMC.end();
  pinMode(PIN_SD_CLK, OUTPUT); digitalWrite(PIN_SD_CLK, LOW); delay(5);
  pinMode(PIN_SD_CLK, INPUT);
  SD_UNLOCK();
  delay(50);

  if (g_sdFailStreak >= 2) {
    if      (g_sdFreqKHz > 4000) g_sdFreqKHz = 4000;
    else if (g_sdFreqKHz > 2000) g_sdFreqKHz = 2000;
    else if (g_sdFreqKHz > 1000) g_sdFreqKHz = 1000;
  }

  ok = mountSdmmc();
  if (ok && g_currentPath.length()) {
    String why; ok = openFseq(g_currentPath, why);
    Serial.printf("[SD] Reopen after remount: %s\n", ok?"OK": why.c_str());
  }

  if (ok) g_sdFailStreak = 0;
  return ok;
}

/* -------------------- Server, Setup, Loop -------------------- */
static void startWifiAP(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  updateHostname();
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  WiFi.setSleep(false); // keep AP awake during SD I/O
  g_stationReconnectScheduled = false;
  if (g_wifiSsid.length()) {
    startStationConnection();
  } else {
    g_stationConnected = false;
    g_stationConnectPending = false;
  }
  if (MDNS.begin(g_stationId.c_str())) MDNS.addService("http","tcp",80);

  // Control + status
  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/status",  HTTP_GET,  handleStatus);

  // Playback & settings
  server.on("/play",    HTTP_GET,  handlePlayLink);
  server.on("/b",       HTTP_POST, handleB);
  server.on("/start",   HTTP_GET,  handleStart);
  server.on("/stop",    HTTP_POST, handleStop);
  server.on("/speed",   HTTP_POST, handleSpeed);
  server.on("/mapcfg",  HTTP_POST, handleMapCfg);
  server.on("/wifi",    HTTP_POST, handleWifiConfig);

  // Diagnostics
  server.on("/fseq/header", HTTP_GET,  handleFseqHeader);
  server.on("/fseq/cblocks",HTTP_GET,  handleCBlocks);
  server.on("/sd/reinit",   HTTP_POST, handleSdReinit);

  // Files
  server.on("/files",   HTTP_GET,  handleFiles);
  server.on("/dl",      HTTP_GET,  handleDownload);
  server.on("/rm",      HTTP_GET,  handleDelete);
  server.on("/mkdir",   HTTP_GET,  handleMkdir);
  server.on("/ren",     HTTP_GET,  handleRename);

  // Upload
  server.on("/upload",  HTTP_POST, handleUploadDone, handleUploadData);
  server.on("/ota",     HTTP_POST, handleOtaApply);

  server.onNotFound([](){ server.send(404, "text/plain", String("404 Not Found: ") + server.uri()); });
  server.begin();
  Serial.println("[HTTP] WebServer listening on :80");
}

void setup(){
  Serial.begin(115200);
  delay(800);
  Serial.println("\n[POV] SK9822 spinner — FSEQ v2 (sparse + zlib per-frame)");
  Serial.println(F("[Quadrant self-check]"));
  for (int k = 0; k < activeArmCount(); ++k) {
    int s0 = armSpokeIdx0(k, spoke1BasedToIdx0(START_SPOKE_1BASED, SPOKES), SPOKES, activeArmCount());
    Serial.printf("Arm %d → spoke %d\n", k+1, s0 + 1);
  }
  Serial.printf("[MAP] labelMode=%d\n", (int)gLabelMode);

  // Restore settings
  prefs.begin("display", false);

  if (prefs.isKey("brightness")) {
    g_brightnessPercent = prefs.getUChar("brightness");
  } else {
    g_missing.brightness = true;
    g_brightnessPercent = 25;
  }

  if (prefs.isKey("fps")) {
    g_fps = prefs.getUShort("fps");
    if (!g_fps) g_fps = 40;
  } else {
    g_missing.fps = true;
    g_fps = 40;
  }

  if (prefs.isKey("startch")) {
    g_startChArm1 = prefs.getULong("startch");
  } else {
    g_missing.startCh = true;
    g_startChArm1 = 1;
  }

  if (prefs.isKey("spokes")) {
    g_spokesTotal = prefs.getUShort("spokes");
    if (!g_spokesTotal) g_spokesTotal = 1;
  } else {
    g_missing.spokes = true;
    g_spokesTotal = 40;
  }

  if (prefs.isKey("arms")) {
    g_armCount = clampArmCount(prefs.getUChar("arms"));
  } else {
    g_missing.arms = true;
    g_armCount = MAX_ARMS;
  }

  if (prefs.isKey("pixels")) {
    g_pixelsPerArm = clampPixelsPerArm(prefs.getUShort("pixels"));
  } else {
    g_missing.pixels = true;
    g_pixelsPerArm = DEFAULT_PIXELS_PER_ARM;
  }

  if (prefs.isKey("stride")) {
    g_strideMode = (StrideMode)prefs.getUChar("stride");
  } else {
    g_missing.stride = true;
    g_strideMode = STRIDE_SPOKE;
  }

  if (prefs.isKey("wssid")) {
    char buf[65]; size_t len = prefs.getString("wssid", buf, sizeof(buf));
    g_wifiSsid = String(buf);
    if (len == 0) g_missing.wifiSsid = true;
  } else {
    g_missing.wifiSsid = true;
    g_wifiSsid = "";
  }

  if (prefs.isKey("wpass")) {
    char buf[97]; prefs.getString("wpass", buf, sizeof(buf));
    g_wifiPass = String(buf);
  } else {
    g_missing.wifiPass = true;
    g_wifiPass = "";
  }

  if (prefs.isKey("wsid")) {
    char buf[65]; size_t len = prefs.getString("wsid", buf, sizeof(buf));
    if (len == 0) {
      g_stationId = DEFAULT_STATION_ID;
      g_missing.stationId = true;
    } else {
      g_stationId = String(buf);
    }
  } else {
    g_missing.stationId = true;
    g_stationId = DEFAULT_STATION_ID;
  }

  computeDerivedSettings();

  Serial.printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
  Serial.printf("[MAP] startCh(Arm1)=%lu spokes=%u arms=%u pixels/arm=%u stride=%s\n",
                (unsigned long)g_startChArm1, g_spokesTotal, (unsigned)activeArmCount(),
                (unsigned)g_pixelsPerArm, (g_strideMode==STRIDE_SPOKE?"SPOKE":"LED"));

  // Create SD mutex before any FS work
  g_sdMutex = xSemaphoreCreateMutex();

  startWifiAP();

  if (!cardPresent()) {
    Serial.printf("[SD] No card (CD HIGH on GPIO%d); UI still available.\n", PIN_SD_CD);
  }
  bool sdOk = mountSdmmc();
  if (!sdOk) {
    Serial.println("[SD] Mount failed; UI still available for diagnostics.");
  } else {
    restoreSettingsFromBackupIfNeeded();
    if (SD_LOCK(pdMS_TO_TICKS(2000))) {
      uint8_t type = SD_MMC.cardType();
      uint64_t sizeMB = (type==CARD_NONE) ? 0 : (SD_MMC.cardSize() / (1024ULL*1024ULL));
      SD_UNLOCK();
      Serial.printf("[SD] Type=%u  Size=%llu MB\n", (unsigned)type, (unsigned long long)sizeMB);
    }
    saveSettingsBackup();
  }

  rebuildStrips();
  blackoutAll();

  g_bootMs   = millis();
  g_playing  = false;
  g_currentPath = "";
  Serial.println("[STATE] Waiting for selection via web UI (5-min timeout to /test2.fseq)");
}

void loop(){
  server.handleClient();
  manageWifiStation();

  if (!g_playing && (millis() - g_bootMs > SELECT_TIMEOUT_MS)) {
    String why;
    if (openFseq("/test2.fseq", why)) {
      Serial.println("[TIMEOUT] Auto-start /test2.fseq");
    } else {
      Serial.printf("[TIMEOUT] open fail: %s\n", why.c_str());
      g_bootMs = millis(); // try again later
    }
  }

  if (!g_playing || g_paused) { delay(1); return; }

  const uint32_t now = millis();
  if (now - g_lastTickMs < g_framePeriodMs) return;
  g_lastTickMs = now;

  if (!renderFrame(g_frameIndex)) {
    ++g_sdFailStreak;
    Serial.printf("[PLAY] frame read failed — streak=%d\n", g_sdFailStreak);
    if (!recoverSd("frame read failed")) {
      if (g_sdFailStreak >= 6) {  // give up gracefully for now
        Serial.println("[SD] Unrecoverable — pausing playback.");
        g_playing = false;
        g_sdFailStreak = 0;
      }
    }
    return;
  }

  g_sdFailStreak = 0; // success resets streak
  g_frameIndex = (g_frameIndex + 1) % (g_fh.frameCount ? g_fh.frameCount : 1);
}
