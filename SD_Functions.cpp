#include "SD_Functions.h"

#include <SD_MMC.h>
#include <Update.h>
#include <WebServer.h>
#include <Preferences.h>

#include "HtmlUtils.h"
#include "WebPages.h"

// Hardware pins
const int PIN_SD_CLK = 10;
const int PIN_SD_CMD = 9;
const int PIN_SD_D0  = 8;
const int PIN_SD_D1  = 13;
const int PIN_SD_D2  = 12;
const int PIN_SD_D3  = 11;
const int PIN_SD_CD  = 14;  // LOW = inserted

// SD backup paths
static const char* const SETTINGS_DIR  = "/config";
static const char* const SETTINGS_FILE = "/config/settings.ini";
static const char* const OTA_FILE      = "/firmware.bin";
static const char* const OTA_FAIL_FILE = "/firmware.failed";

static const uint32_t SD_FREQ_OPTIONS[] = { 8000, 4000, 2000, 1000, 400 };
static const size_t   SD_FREQ_OPTION_COUNT = sizeof(SD_FREQ_OPTIONS) / sizeof(SD_FREQ_OPTIONS[0]);

SemaphoreHandle_t g_sdMutex = nullptr;
SdBusPreference   g_sdPreferredBusWidth = SD_BUS_AUTO;
uint32_t          g_sdBaseFreqKHz       = 8000;
uint32_t          g_sdFreqKHz           = 8000;
int               g_sdFailStreak        = 0;
bool              g_sdReady             = false;
uint8_t           g_sdBusWidth          = 0;

// Externs from main sketch
extern WebServer server;
extern Preferences prefs;
extern String g_staSsid;
extern String g_staPass;
extern String g_stationId;
extern bool g_autoplayEnabled;
extern bool g_watchdogEnabled;
extern bool g_bgEffectEnabled;
extern bool g_bgEffectActive;
extern String g_bgEffectPath;
extern uint32_t g_bgEffectNextAttemptMs;
enum OutputMode : uint8_t { OUT_SPI = 0, OUT_PARALLEL = 1 };
extern uint8_t g_outputMode;
extern uint8_t g_brightnessPercent;
extern uint8_t g_brightness;
extern uint16_t g_fps;
extern uint32_t g_startChArm1;
extern uint16_t g_spokesTotal;
extern uint8_t g_armCount;
extern uint16_t g_pixelsPerArm;
extern StrideMode g_strideMode;
extern bool g_playing;
extern bool g_paused;
extern uint32_t g_bootMs;
extern bool g_hallDiagEnabled;
extern String g_currentPath;

extern bool openFseq(const String& path, String& why);
extern void feedWatchdog();

namespace {

bool ensureSettingsDirLocked() {
  if (!SD_MMC.exists(SETTINGS_DIR)) {
    if (!SD_MMC.mkdir(SETTINGS_DIR)) {
      Serial.println("[CFG] mkdir /config failed");
      return false;
    }
  }
  return true;
}

bool ensureBgEffectsDirLocked() {
  if (!SD_MMC.exists(BG_EFFECTS_DIR)) {
    if (!SD_MMC.mkdir(BG_EFFECTS_DIR)) {
      Serial.println("[CFG] mkdir /BGEffects failed");
      return false;
    }
  }
  return true;
}

bool saveSettingsBackupLocked() {
  if (!ensureSettingsDirLocked()) return false;
  SD_MMC.remove(SETTINGS_FILE);
  File f = SD_MMC.open(SETTINGS_FILE, FILE_WRITE);
  if (!f) {
    Serial.println("[CFG] open settings.ini failed");
    return false;
  }
  f.print("brightness="); f.println((unsigned)g_brightnessPercent);
  f.print("fps=");        f.println((unsigned)g_fps);
  f.print("startch=");    f.println((unsigned long)g_startChArm1);
  f.print("spokes=");     f.println((unsigned)g_spokesTotal);
  f.print("arms=");       f.println((unsigned)g_armCount);
  f.print("pixels=");     f.println((unsigned)g_pixelsPerArm);
  f.print("stride=");     f.println((unsigned)g_strideMode);
  f.print("ssid=");       f.println(g_staSsid);
  f.print("pass=");       f.println(g_staPass);
  f.print("station=");    f.println(g_stationId);
  f.print("sdmode=");     f.println((unsigned)g_sdPreferredBusWidth);
  f.print("sdfreq=");     f.println((unsigned long)g_sdBaseFreqKHz);
  f.print("autoplay=");   f.println(g_autoplayEnabled ? 1 : 0);
  f.print("watchdog=");   f.println(g_watchdogEnabled ? 1 : 0);
  f.print("bge_enable="); f.println(g_bgEffectEnabled ? 1 : 0);
  f.print("bge_path=");   f.println(g_bgEffectPath);
  f.print("outmode=");    f.println((unsigned)g_outputMode);
  f.close();
  return true;
}

bool loadSettingsBackupLocked(SettingsData &out) {
  File f = SD_MMC.open(SETTINGS_FILE, FILE_READ);
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.replace("\r", "");
    if (!line.length()) continue;
    int eq = line.indexOf('=');
    if (eq <= 0) continue;
    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);
    if (key == "brightness") { out.hasBrightness = true; out.brightness = (uint8_t)clampU32(value.toInt(),0,100); }
    else if (key == "fps")   { out.hasFps = true; out.fps = (uint16_t)clampU32(value.toInt(),1,120); }
    else if (key == "startch") { out.hasStartCh = true; out.startCh = (uint32_t)strtoul(value.c_str(), nullptr, 10); }
    else if (key == "spokes") { out.hasSpokes = true; out.spokes = (uint16_t)clampU32(value.toInt(),1,65535); }
    else if (key == "arms")   { out.hasArms = true; out.arms = clampArmCount(value.toInt()); }
    else if (key == "pixels") { out.hasPixels = true; out.pixels = clampPixelsPerArm(value.toInt()); }
    else if (key == "stride") { out.hasStride = true; out.stride = (uint8_t)clampU32(value.toInt(),0,1); }
    else if (key == "ssid")   { out.hasStaSsid = true; out.staSsid = value; }
    else if (key == "pass")   { out.hasStaPass = true; out.staPass = value; }
    else if (key == "station") { out.hasStation = true; out.stationId = value; }
    else if (key == "sdmode") { out.hasSdMode = true; out.sdMode = (uint8_t)clampU32(value.toInt(),0,4); }
    else if (key == "sdfreq") { out.hasSdFreq = true; out.sdFreq = (uint32_t)strtoul(value.c_str(), nullptr, 10); }
    else if (key == "autoplay") { out.hasAutoplay = true; out.autoplay = (value.toInt() != 0); }
    else if (key == "watchdog") { out.hasWatchdog = true; out.watchdog = (value.toInt() != 0); }
    else if (key == "bge_enable") { out.hasBgEffectEnable = true; out.bgEffectEnable = (value.toInt() != 0); }
    else if (key == "bge_path") { out.hasBgEffectPath = true; out.bgEffectPath = value; }
    else if (key == "outmode") { out.hasOutMode = true; out.outMode = (uint8_t)clampU32(value.toInt(),0,1); }
  }
  f.close();
  return true;
}

} // namespace

bool SD_LOCK(TickType_t timeout) {
  return g_sdMutex && xSemaphoreTake(g_sdMutex, timeout) == pdTRUE;
}

void SD_UNLOCK() {
  if (g_sdMutex) xSemaphoreGive(g_sdMutex);
}

SdBusPreference sanitizeSdMode(uint8_t mode) {
  if (mode == SD_BUS_1BIT) return SD_BUS_1BIT;
  if (mode == SD_BUS_4BIT) return SD_BUS_4BIT;
  return SD_BUS_AUTO;
}

bool isValidSdFreq(uint32_t freq) {
  for (size_t i = 0; i < SD_FREQ_OPTION_COUNT; ++i) {
    if (SD_FREQ_OPTIONS[i] == freq) return true;
  }
  return false;
}

uint32_t sanitizeSdFreq(uint32_t freq) {
  return isValidSdFreq(freq) ? freq : SD_FREQ_OPTIONS[0];
}

uint32_t nextLowerSdFreq(uint32_t freq) {
  for (size_t i = 0; i < SD_FREQ_OPTION_COUNT; ++i) {
    if (SD_FREQ_OPTIONS[i] == freq) {
      if (i + 1 < SD_FREQ_OPTION_COUNT) return SD_FREQ_OPTIONS[i + 1];
      return SD_FREQ_OPTIONS[i];
    }
  }
  return SD_FREQ_OPTIONS[SD_FREQ_OPTION_COUNT - 1];
}

void persistSettingsToSd() {
  if (!g_sdReady || !g_sdMutex) return;
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return;
  bool ok = saveSettingsBackupLocked();
  SD_UNLOCK();
  if (!ok) Serial.println("[CFG] Failed to persist settings to SD");
}

void ensureSettingsFromBackup(const PrefPresence &present) {
  if (!g_sdReady || !g_sdMutex) return;
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return;
  SettingsData data;
  bool ok = loadSettingsBackupLocked(data);
  SD_UNLOCK();
  if (!ok) return;

  if (!present.brightness && data.hasBrightness) {
    g_brightnessPercent = clampU32(data.brightness, 0, 100);
    g_brightness        = (uint8_t)((255 * g_brightnessPercent) / 100);
    prefs.putUChar("brightness", g_brightnessPercent);
  }
  if (!present.fps && data.hasFps) {
    g_fps = data.fps ? data.fps : 40;
    prefs.putUShort("fps", g_fps);
  }
  if (!present.startCh && data.hasStartCh) {
    g_startChArm1 = data.startCh ? data.startCh : 1;
    prefs.putULong("startch", g_startChArm1);
  }
  if (!present.spokes && data.hasSpokes) {
    g_spokesTotal = data.spokes ? data.spokes : 1;
    prefs.putUShort("spokes", g_spokesTotal);
  }
  if (!present.arms && data.hasArms) {
    g_armCount = clampArmCount(data.arms);
    prefs.putUChar("arms", g_armCount);
  }
  if (!present.pixels && data.hasPixels) {
    g_pixelsPerArm = clampPixelsPerArm(data.pixels);
    prefs.putUShort("pixels", g_pixelsPerArm);
  }
  if (!present.stride && data.hasStride) {
    g_strideMode = (StrideMode)((data.stride == 0) ? STRIDE_SPOKE : STRIDE_LED);
    prefs.putUChar("stride", (uint8_t)g_strideMode);
  }
  if ((!present.staSsid || !g_staSsid.length()) && data.hasStaSsid) {
    g_staSsid = data.staSsid;
    prefs.putString("sta_ssid", g_staSsid);
  }
  if ((!present.staPass || !g_staPass.length()) && data.hasStaPass) {
    g_staPass = data.staPass;
    prefs.putString("sta_pass", g_staPass);
  }
  if ((!present.station || !g_stationId.length()) && data.hasStation) {
    g_stationId = data.stationId;
    prefs.putString("station", g_stationId);
  }
  if (!present.autoplay && data.hasAutoplay) {
    g_autoplayEnabled = data.autoplay;
    prefs.putBool("autoplay", g_autoplayEnabled);
  }
  if (!present.watchdog && data.hasWatchdog) {
    g_watchdogEnabled = data.watchdog;
    prefs.putBool("watchdog", g_watchdogEnabled);
  }
  if (!present.bgEffectEnable && data.hasBgEffectEnable) {
    g_bgEffectEnabled = data.bgEffectEnable;
    prefs.putBool("bge_enable", g_bgEffectEnabled);
  }
  if (!present.bgEffectPath && data.hasBgEffectPath) {
    g_bgEffectPath = sanitizeBgEffectPath(data.bgEffectPath);
    prefs.putString("bge_path", g_bgEffectPath);
    g_bgEffectNextAttemptMs = millis();
  }
  if (!present.sdMode && data.hasSdMode) {
    g_sdPreferredBusWidth = sanitizeSdMode(data.sdMode);
    prefs.putUChar("sdmode", (uint8_t)g_sdPreferredBusWidth);
  }
  if (!present.sdFreq && data.hasSdFreq) {
    g_sdBaseFreqKHz = sanitizeSdFreq(data.sdFreq);
    g_sdFreqKHz = g_sdBaseFreqKHz;
    prefs.putUInt("sdfreq", g_sdBaseFreqKHz);
  }
  if (!present.outMode && data.hasOutMode) {
    g_outputMode = (data.outMode == OUT_PARALLEL) ? OUT_PARALLEL : OUT_SPI;
    prefs.putUChar("outmode", g_outputMode);
  }
}


void checkSdFirmwareUpdate() {
  if (!g_sdReady || !g_sdMutex) return;
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) return;
  File f = SD_MMC.open(OTA_FILE, FILE_READ);
  if (!f) { SD_UNLOCK(); return; }
  size_t size = f.size();
  if (!size) {
    f.close();
    SD_MMC.remove(OTA_FILE);
    SD_UNLOCK();
    Serial.println("[OTA] Empty firmware.bin removed");
    return;
  }
  Serial.printf("[OTA] Found %s (%u bytes)\n", OTA_FILE, (unsigned)size);
  if (!Update.begin(size)) {
    Serial.printf("[OTA] Update begin failed: %s\n", Update.errorString());
    f.close();
    SD_MMC.remove(OTA_FAIL_FILE);
    SD_MMC.rename(OTA_FILE, OTA_FAIL_FILE);
    SD_UNLOCK();
    return;
  }
  uint8_t buf[4096];
  while (f.available()) {
    size_t rd = f.read(buf, sizeof(buf));
    if (!rd) break;
    if (Update.write(buf, rd) != rd) {
      Serial.printf("[OTA] Write failed: %s\n", Update.errorString());
      Update.end();
      f.close();
      SD_MMC.remove(OTA_FAIL_FILE);
      SD_MMC.rename(OTA_FILE, OTA_FAIL_FILE);
      SD_UNLOCK();
      return;
    }
    feedWatchdog();
  }
  f.close();
  if (!Update.end()) {
    Serial.printf("[OTA] Update end failed: %s\n", Update.errorString());
    SD_MMC.remove(OTA_FAIL_FILE);
    SD_MMC.rename(OTA_FILE, OTA_FAIL_FILE);
    SD_UNLOCK();
    return;
  }
  if (!Update.isFinished()) {
    Serial.println("[OTA] Update incomplete");
    SD_MMC.remove(OTA_FAIL_FILE);
    SD_MMC.rename(OTA_FILE, OTA_FAIL_FILE);
    SD_UNLOCK();
    return;
  }
  Serial.println("[OTA] Update successful, rebooting...");
  SD_MMC.remove(OTA_FILE);
  SD_UNLOCK();
  delay(200);
  ESP.restart();
}

bool cardPresent() {
  pinMode(PIN_SD_CD, INPUT_PULLUP);
  return digitalRead(PIN_SD_CD) == LOW;
}

namespace {
void sd_preflight() {
  pinMode(PIN_SD_CMD, INPUT_PULLUP);
  pinMode(PIN_SD_D0,  INPUT_PULLUP);
  pinMode(PIN_SD_D1,  INPUT_PULLUP);
  pinMode(PIN_SD_D2,  INPUT_PULLUP);
  pinMode(PIN_SD_D3,  INPUT_PULLUP);
  delay(2);
  Serial.printf("[SD] Preflight CMD@%d=%d  D0@%d=%d  D1@%d=%d  D2@%d=%d  D3@%d=%d (expect 1s)\n",
                PIN_SD_CMD, digitalRead(PIN_SD_CMD),
                PIN_SD_D0,  digitalRead(PIN_SD_D0),
                PIN_SD_D1,  digitalRead(PIN_SD_D1),
                PIN_SD_D2,  digitalRead(PIN_SD_D2),
                PIN_SD_D3,  digitalRead(PIN_SD_D3));
}
}

bool mountSdmmc() {
  if (!SD_LOCK(pdMS_TO_TICKS(2000))) {
    Serial.println("[SD] mount lock timeout");
    return false;
  }
  g_sdFreqKHz = sanitizeSdFreq(g_sdFreqKHz);
  g_sdBaseFreqKHz = sanitizeSdFreq(g_sdBaseFreqKHz);

  uint8_t attempts[2] = { 4, 1 };
  size_t attemptCount = 0;
  if (g_sdPreferredBusWidth == SD_BUS_AUTO) { attempts[0] = 4; attempts[1] = 1; attemptCount = 2; }
  else if (g_sdPreferredBusWidth == SD_BUS_4BIT) { attempts[0] = 4; attemptCount = 1; }
  else { attempts[0] = 1; attemptCount = 1; }

  bool ok = false;
  sd_preflight();
  g_sdBusWidth = 0;

  for (size_t i=0; i<attemptCount; ++i) {
    feedWatchdog();
    uint8_t mode = attempts[i];
    if (i > 0) { SD_MMC.end(); delay(2); }
    if (mode == 4) {
      SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
      ok = SD_MMC.begin("/sdcard", false /*4-bit*/, false /*no-format*/, g_sdFreqKHz);
    } else {
      SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, -1, -1, -1);
      ok = SD_MMC.begin("/sdcard", true /*1-bit*/, false /*no-format*/, g_sdFreqKHz);
    }
    Serial.printf("[SD] Mounted (%ubit request=%u) @ %lu kHz: %s\n",
                  (unsigned)mode, (unsigned)g_sdPreferredBusWidth,
                  (unsigned long)g_sdFreqKHz, ok?"OK":"FAIL");
    if (ok) { g_sdBusWidth = mode; break; }
  }
  SD_UNLOCK();
  g_sdReady = ok;
  return ok;
}

namespace {
File   g_uploadFile;
String g_uploadFilename;
size_t g_uploadBytes = 0;
}

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

void listFseqInDir(const char* path, String& optionsHtml, uint8_t depth) {
  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { Serial.println("[SD] busy; skip list"); return; }
  listFseqInDir_locked(path, optionsHtml, depth);
  SD_UNLOCK();
}

static void listBgEffects_locked(String& optionsHtml, const String& current) {
  File dir = SD_MMC.open(BG_EFFECTS_DIR);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
  File ent;
  while ((ent = dir.openNextFile())) {
    if (ent.isDirectory()) { ent.close(); continue; }
    String name = ent.name();
    if (!isFseqName(name)) { ent.close(); continue; }
    bool selected = (name == current);
    String valueEsc = htmlEscape(name);
    String labelEsc = htmlEscape(bgEffectDisplayName(name));
    optionsHtml += "<option value='";
    optionsHtml += valueEsc;
    optionsHtml += "'";
    if (selected) optionsHtml += " selected";
    optionsHtml += ">";
    optionsHtml += labelEsc;
    optionsHtml += "</option>";
    ent.close();
  }
  dir.close();
}

void listBgEffects(String& optionsHtml, const String& current) {
  optionsHtml += "<option value=''";
  if (!current.length()) optionsHtml += " selected";
  optionsHtml += ">(none)</option>";
  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { Serial.println("[SD] busy; skip bge list"); return; }
  listBgEffects_locked(optionsHtml, current);
  SD_UNLOCK();
}

void handleFiles() {
  String path = server.hasArg("path") ? server.arg("path") : "/";
  if (!path.length() || path[0] != '/') path = "/" + path;

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
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

  String pathAttrEsc = htmlEscape(path);
  String backAttrEsc = htmlEscape(backPlain);
  String html = WebPages::filesPageHeader(pathEsc, parentEnc, pathEnc, backEncoded, pathAttrEsc, backAttrEsc);

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

void handleDownload() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path"); if (!path.startsWith("/")) path = "/" + path;

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(5000))) { server.send(503,"text/plain","SD busy"); return; }
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) { if (f) f.close(); SD_UNLOCK(); server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + htmlEscape(path.substring(path.lastIndexOf('/')+1)) + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
  SD_UNLOCK();
}

void handlePlayLink() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.exists(path) && isFseqName(path);
  SD_UNLOCK();
  if (!ok) { server.sendHeader("Location", back); server.send(302, "text/plain", "Not a .fseq or missing"); return; }

  String why; openFseq(path, why);
  server.sendHeader("Location", back);
  server.send(302, "text/plain", "OK");
}

void handleDelete() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
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

void handleMkdir() {
  if (!server.hasArg("path") || !server.hasArg("name")) { server.send(400, "text/plain", "args"); return; }
  String base = server.hasArg("path") ? server.arg("path") : "/";
  String name = server.arg("name");
  if (!base.startsWith("/")) base = "/" + base;
  if (name.indexOf('/')>=0 || !name.length()) { server.send(400, "text/plain", "bad name"); return; }
  if (!base.endsWith("/")) base += "/";

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.mkdir(base + name);
  SD_UNLOCK();

  server.sendHeader("Location", String("/files?path=") + urlEncode(base.substring(0, base.length()-1)));
  server.send(ok?302:500, "text/plain", ok?"Created":"Create failed");
}

void handleRename() {
  if (!server.hasArg("path") || !server.hasArg("to")) { server.send(400, "text/plain", "args"); return; }
  String p = server.arg("path");
  String to = server.arg("to");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!p.startsWith("/")) p = "/" + p;
  if (to.indexOf('/')>=0 || !to.length()) { server.sendHeader("Location", back); server.send(302, "text/plain", "bad name"); return; }

  String dir = dirnameOf(p);
  String dst = (dir == "/") ? ("/" + to) : (dir + "/" + to);

  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.sendHeader("Location", back); server.send(302,"text/plain","SD busy"); return; }
  bool ok = SD_MMC.rename(p, dst);
  SD_UNLOCK();

  server.sendHeader("Location", back);
  server.send(ok?302:500, "text/plain", ok?"Renamed":"Rename failed");
}

void handleUploadData() {
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
      if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(5000))) { Serial.println("[UPLOAD] SD busy"); return; }
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
      if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) {
        g_uploadFile.write(up.buf, up.currentSize);
        SD_UNLOCK();
        g_uploadBytes += up.currentSize;
        feedWatchdog();
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_uploadFile) {
      if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) { g_uploadFile.close(); SD_UNLOCK(); }
      Serial.printf("[UPLOAD] DONE %s (%u bytes)\n", g_uploadFilename.c_str(), (unsigned)g_uploadBytes);
    } else {
      Serial.println("[UPLOAD] Aborted/invalid file");
    }
  }
}

void handleUploadDone() {
  String back = server.hasArg("back") ? server.arg("back") : "/";
  bool ok=false;
  if (isFseqName(g_uploadFilename)) {
    if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) {
      ok = SD_MMC.exists(g_uploadFilename);
      SD_UNLOCK();
    }
  }
  if (!isFseqName(g_uploadFilename)) {
    server.send(415, "text/html", WebPages::uploadRejectedPage(back));
    return;
  }
  if (!ok) {
    server.send(500, "text/html", WebPages::uploadFailurePage(back));
    return;
  }
  server.send(200, "text/html", WebPages::uploadSuccessPage(back, g_uploadFilename, g_uploadBytes));
}

void handleSdReinit() {
  bool ok=false;
  if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(2000))) { server.send(503,"text/plain","SD busy"); return; }
  bool mounted = (SD_MMC.cardType()!=CARD_NONE);
  SD_UNLOCK();

  if (mounted) { g_sdReady = true; ok = true; }
  else { ok = mountSdmmc(); }

  if (!ok) { server.send(500,"text/plain","SD not present"); return; }
  if (!g_currentPath.length()) { server.send(200,"text/plain","SD OK; no file"); return; }
  String why;
  if (openFseq(g_currentPath, why)) server.send(200,"text/plain","SD OK; file reopened");
  else server.send(500,"text/plain", String("reopen fail: ")+why);
}

void handleSdConfig() {
  if (!server.hasArg("mode") || !server.hasArg("freq")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing parameters\"}");
    return;
  }

  String modeStr = server.arg("mode");
  String freqStr = server.arg("freq");

  char *end=nullptr;
  long modeVal = strtol(modeStr.c_str(), &end, 10);
  if (end == modeStr.c_str() || *end != '\0') { server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid mode\"}"); return; }
  if (!(modeVal == 0 || modeVal == 1 || modeVal == 4)) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"unsupported mode\"}"); return; }
  SdBusPreference newMode = sanitizeSdMode((uint8_t)modeVal);

  end = nullptr;
  uint32_t freqVal = (uint32_t)strtoul(freqStr.c_str(), &end, 10);
  if (end == freqStr.c_str() || *end != '\0' || !isValidSdFreq(freqVal)) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid frequency\"}"); return; }

  bool changed = false;
  if (newMode != g_sdPreferredBusWidth) { g_sdPreferredBusWidth = newMode; prefs.putUChar("sdmode", (uint8_t)g_sdPreferredBusWidth); changed = true; }
  if (freqVal != g_sdBaseFreqKHz) { g_sdBaseFreqKHz = sanitizeSdFreq(freqVal); g_sdFreqKHz = g_sdBaseFreqKHz; prefs.putUInt("sdfreq", g_sdBaseFreqKHz); changed = true; }
  if (changed) { g_sdFailStreak = 0; persistSettingsToSd(); }

  bool card = cardPresent();
  bool remounted = false;
  bool reopened = false;
  if (card) {
    if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) { SD_MMC.end(); g_sdBusWidth = 0; g_sdReady = false; SD_UNLOCK(); }
    delay(20);
    g_sdFreqKHz = g_sdBaseFreqKHz;
    remounted = mountSdmmc();
    if (remounted) {
      g_sdFailStreak = 0;
      if (g_currentPath.length()) {
        String why;
        reopened = openFseq(g_currentPath, why);
        if (!reopened && why.length()) Serial.printf("[SD] reopen after config failed: %s\n", why.c_str());
      }
    }
  } else { g_sdReady = false; g_sdBusWidth = 0; }

  String json = "{\"ok\":true";
  json += ",\"ready\":" + String(g_sdReady?"true":"false");
  json += ",\"currentWidth\":" + String((unsigned)g_sdBusWidth);
  json += ",\"desiredMode\":" + String((unsigned)g_sdPreferredBusWidth);
  json += ",\"baseFreq\":" + String((unsigned long)g_sdBaseFreqKHz);
  json += ",\"freq\":" + String((unsigned long)g_sdFreqKHz);
  if (remounted) json += ",\"remounted\":true";
  if (reopened) json += ",\"fileReopened\":true";
  json += "}";
  server.send(200, "application/json", json);
}

namespace {
bool otaAuthOK() { return true; } // stub for future auth

File   g_otaFile;
size_t g_otaBytes = 0;
File   g_fwSdFile;
size_t g_fwSdBytes = 0;
}

void handleOtaPage() {
  server.send(200, "text/html; charset=utf-8", WebPages::directOtaPage());
}

void handleOtaData() {
  if (!otaAuthOK()) return;
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_otaBytes = 0;
    Serial.printf("[OTA] Direct start: %s\n", up.filename.c_str());
    if (!Update.begin()) { Serial.printf("[OTA] begin failed: %s\n", Update.errorString()); }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.isRunning()) {
      size_t w = Update.write(up.buf, up.currentSize);
      if (w != up.currentSize) Serial.printf("[OTA] write failed: %s\n", Update.errorString());
    }
    g_otaBytes += up.currentSize;
    feedWatchdog();
  } else if (up.status == UPLOAD_FILE_END) {
    bool ok = Update.end(true);
    Serial.printf("[OTA] Direct end (%u bytes): %s\n", (unsigned)g_otaBytes, ok?"OK":"FAIL");
  }
}

void handleOtaFinish() {
  if (!otaAuthOK()) { server.send(401,"text/plain","Unauthorized"); return; }
  if (Update.isFinished()) { server.send(200, "text/plain", "OTA complete, rebooting..."); delay(200); ESP.restart(); }
  else { server.send(500, "text/plain", String("OTA failed: ") + Update.errorString()); }
}

void handleFwUploadData() {
  if (!otaAuthOK()) return;
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_fwSdBytes = 0;
    if (!g_sdMutex || !SD_LOCK(pdMS_TO_TICKS(5000))) { Serial.println("[FWSD] SD busy at START"); return; }
    if (SD_MMC.exists(OTA_FILE)) SD_MMC.remove(OTA_FILE);
    g_fwSdFile = SD_MMC.open(OTA_FILE, FILE_WRITE);
    SD_UNLOCK();
    Serial.println("[FWSD] START -> /firmware.bin");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_fwSdFile) {
      if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) {
        g_fwSdFile.write(up.buf, up.currentSize);
        SD_UNLOCK();
        g_fwSdBytes += up.currentSize;
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_fwSdFile) {
      if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) { g_fwSdFile.close(); SD_UNLOCK(); }
      Serial.printf("[FWSD] DONE (%u bytes)\n", (unsigned)g_fwSdBytes);
    } else {
      Serial.println("[FWSD] Aborted/invalid");
    }
  }
}

void handleFwUploadDone() {
  if (!otaAuthOK()) { server.send(401,"text/plain","Unauthorized"); return; }
  bool present = false;
  if (g_sdMutex && SD_LOCK(pdMS_TO_TICKS(2000))) { present = SD_MMC.exists(OTA_FILE); SD_UNLOCK(); }
  server.sendHeader("Location", present ? "/updates?uploaded=1" : "/updates?uploaded=0");
  server.send(302);
}

void handleUpdatesPage() {
  bool canReboot = (server.hasArg("uploaded") && server.arg("uploaded") == "1");
  String html = WebPages::updatesPage(canReboot);
  server.send(200, "text/html; charset=utf-8", html);
}
