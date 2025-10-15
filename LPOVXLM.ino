#include <Arduino.h>
#include <SD_MMC.h>
#include <Adafruit_DotStar.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdlib.h>
#include <string.h>
#include "QuadMap.h"

#include "WebPages.h"

/* -------------------- Optional zlib backends (auto-detect) -------------------- */
#if defined(__has_include)
  #if __has_include(<miniz.h>)
    #include <miniz.h>
  #endif
  #if __has_include(<zlib.h>)
    #include <zlib.h>
  #endif
#endif
/* ----------------------------------------------------------------------------- */

// ---------- SK9822 / APA102 ----------
static const uint8_t  MAX_ARMS               = 4;
static const uint16_t DEFAULT_PIXELS_PER_ARM = 144;
static const uint16_t MAX_PIXELS_PER_ARM     = 1024;
static const int ARM_CLK[MAX_ARMS]  = { 18, 38, 36, 48 };
static const int ARM_DATA[MAX_ARMS] = { 17, 39, 35, 45 };

// ---------- SD-MMC pins (1-bit) ----------
static const int PIN_SD_CLK = 14;
static const int PIN_SD_CMD = 15;
static const int PIN_SD_D0  = 2;
// keep defined in case you flip back to 4-bit later
static const int PIN_SD_D1  = 4;
static const int PIN_SD_D2  = 12;
static const int PIN_SD_D3  = 13;
static const int PIN_SD_CD  = 42;  // LOW = inserted

// === Quadrant mapping controls ===
static int START_SPOKE_1BASED = 1;                    // make this user-settable later
static SpokeLabelMode gLabelMode = FLOOR_TO_BOUNDARY; // matches xLights labels: 1,11,20,30
static const int SPOKES = 40;                         // your xLights radial spokes

// ---------- Wi-Fi AP ----------
static const char* AP_SSID  = "POV-Spinner";
static const char* AP_PASS  = "POV123456";
static const IPAddress AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);
WebServer server(80);

// ---------- Persisted settings (NVS = flash) ----------
Preferences prefs;
uint8_t  g_brightnessPercent = 25;
uint8_t  g_brightness        = 63;
uint16_t g_fps               = 40;
uint32_t g_framePeriodMs     = 25;

// Spinner model mapping (persisted)
uint32_t g_startChArm1   = 1;    // 1-based absolute channel (R of Arm1, Pixel0)
uint16_t g_spokesTotal   = 40;   // total spokes in spinner
uint8_t  g_armCount      = MAX_ARMS;
uint16_t g_pixelsPerArm  = DEFAULT_PIXELS_PER_ARM;

enum StrideMode : uint8_t { STRIDE_SPOKE=0, STRIDE_LED=1 };
StrideMode g_strideMode = STRIDE_SPOKE;  // default SPOKE

// Rotary index (0..spokes-1)
volatile uint16_t g_indexPosition = 0;

// Playback state
volatile bool  g_playing = false, g_paused = false;
String         g_currentPath;
uint32_t       g_frameIndex = 0, g_lastTickMs = 0;
uint32_t       g_bootMs = 0;
const uint32_t SELECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;

Adafruit_DotStar* strips[MAX_ARMS] = { nullptr };

/* -------------------- FSEQ v2 reader (sparse + compression-aware) -------------------- */
struct SparseRange {
  uint32_t start;   // 0-based absolute channel
  uint32_t count;   // channels in this range
  uint32_t accum;   // starting index within frame (prefix sum)
};

struct CompBlock {
  uint32_t uSize;   // uncompressed size (bytes)
  uint32_t cSize;   // compressed size (bytes)
};

struct FseqHeader {
  uint16_t chanDataOffset = 0;   // where channel/compressed payload begins
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
SparseRange* g_ranges      = nullptr;     // sparse table
uint8_t*     g_frameBuf    = nullptr;     // size==channelCount
// Compression state
CompBlock*   g_cblocks     = nullptr;
uint32_t     g_compCount   = 0;
uint64_t     g_compBase    = 0;           // start of compressed data
bool         g_compPerFrame= false;       // per-frame zlib (supported)

/* -------------------- Small helpers -------------------- */
static inline uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi){ if (v<lo) return lo; if (v>hi) return hi; return v; }
static inline int32_t  clampI32(int32_t v, int32_t lo, int32_t hi){ if (v<lo) return lo; if (v>hi) return hi; return v; }
static inline uint8_t  clampArmCount(int32_t v){ if (v<1) return 1; if (v>MAX_ARMS) return MAX_ARMS; return (uint8_t)v; }
static inline uint16_t clampPixelsPerArm(int32_t v){ if (v<1) return 1; if (v>MAX_PIXELS_PER_ARM) return MAX_PIXELS_PER_ARM; return (uint16_t)v; }
static inline uint8_t  activeArmCount(){ return (g_armCount < 1) ? 1 : ((g_armCount > MAX_ARMS) ? MAX_ARMS : g_armCount); }

static bool isFseqName(const String& n) {
  int dot = n.lastIndexOf('.');
  if (dot < 0) return false;
  String ext = n.substring(dot + 1);
  ext.toLowerCase();
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
  if (slash >= 0) base = base.substring(slash+1); // strip any directories
  return (d == "/") ? ("/" + base) : (d + "/" + base);
}

/* -------------------- SD helpers -------------------- */
static bool cardPresent(){ pinMode(PIN_SD_CD, INPUT_PULLUP); return digitalRead(PIN_SD_CD)==LOW; }
static bool mountSdmmc(){
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3);
  const uint32_t FREQ_KHZ = 10000; // 10 MHz, 1-bit
  return SD_MMC.begin("/sdcard", true /*1-bit*/, false /*no-format*/, FREQ_KHZ);
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
#if defined(MZ_OK) /* miniz present */
  mz_ulong dst = (mz_ulong)out_len;
  int rc = mz_uncompress((unsigned char*)out, &dst,
                         (const unsigned char*)in, (mz_ulong)in_len);
  return (rc == MZ_OK) && (dst == (mz_ulong)out_len);
#elif defined(Z_OK) /* zlib present */
  uLongf dst = (uLongf)out_len;
  int rc = uncompress((Bytef*)out, &dst, (const Bytef*)in, (uLong)in_len);
  return (rc == Z_OK) && (dst == (uLongf)out_len);
#else
  (void)in; (void)in_len; (void)out; (void)out_len;
  return false;
#endif
}

/* -------------------- FSEQ open/close/load -------------------- */
static void freeFseq(){
  if (g_fseq) g_fseq.close();
  if (g_ranges){ free(g_ranges); g_ranges=nullptr; }
  if (g_frameBuf){ free(g_frameBuf); g_frameBuf=nullptr; }
  if (g_cblocks){ free(g_cblocks); g_cblocks=nullptr; }
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
  g_fseq = SD_MMC.open(path, FILE_READ);
  if (!g_fseq){ why="open fail"; return false; }

  // Magic
  uint8_t magic[4];
  if (g_fseq.read(magic,4)!=4){ why="short"; return false; }
  if (!((magic[0]=='F'||magic[0]=='P') && magic[1]=='S' && magic[2]=='E' && magic[3]=='Q')){
    why="bad magic"; return false;
  }
  if (!readU16(g_fseq, g_fh.chanDataOffset)) { why="hdr"; return false; }
  if (g_fseq.read(&g_fh.minor,1)!=1) { why="hdr"; return false; }
  if (g_fseq.read(&g_fh.major,1)!=1) { why="hdr"; return false; }
  if (!readU16(g_fseq, g_fh.varDataOffset)) { why="hdr"; return false; }
  if (!readU32(g_fseq, g_fh.channelCount))  { why="hdr"; return false; }
  if (!readU32(g_fseq, g_fh.frameCount))    { why="hdr"; return false; }
  if (g_fseq.read(&g_fh.stepTimeMs,1)!=1)   { why="hdr"; return false; }
  if (g_fseq.read(&g_fh.flags,1)!=1)        { why="hdr"; return false; }

  uint8_t ecct_ct_scc_res[4];
  if (g_fseq.read(ecct_ct_scc_res,4)!=4)    { why="hdr"; return false; }
  g_fh.compType     = (ecct_ct_scc_res[0] & 0x0F);
  g_fh.compBlockCnt = ecct_ct_scc_res[1];
  g_fh.sparseCnt    = ecct_ct_scc_res[2];
  if (!readU64(g_fseq, g_fh.uniqueId))      { why="hdr"; return false; }

  // Compression block table (optional)
  if (g_fh.compBlockCnt > 0) {
    g_cblocks = (CompBlock*)malloc(sizeof(CompBlock)*g_fh.compBlockCnt);
    if (!g_cblocks){ why="oom ctab"; return false; }
    for (uint32_t i=0;i<g_fh.compBlockCnt;++i){
      if (!readU32(g_fseq, g_cblocks[i].uSize)) { why="ctab"; return false; }
      if (!readU32(g_fseq, g_cblocks[i].cSize)) { why="ctab"; return false; }
    }
    g_compCount = g_fh.compBlockCnt;
  }

  // Sparse table (optional)
  if (g_fh.sparseCnt > 0){
    g_ranges = (SparseRange*)malloc(sizeof(SparseRange)*g_fh.sparseCnt);
    if (!g_ranges){ why="oom ranges"; return false; }
    uint32_t accum=0;
    for (uint8_t i=0;i<g_fh.sparseCnt;++i){
      uint8_t b[6]; if (g_fseq.read(b,6)!=6){ why="ranges"; return false; }
      uint32_t start = (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16);
      uint32_t count = (uint32_t)b[3]|((uint32_t)b[4]<<8)|((uint32_t)b[5]<<16);
      g_ranges[i] = { start, count, accum };
      accum += count;
    }
  }

  // Jump to payload
  g_fseq.seek(g_fh.chanDataOffset, SeekSet);

  // Compression detection
  if (g_fh.compType == 0) {
    g_compBase = g_fh.chanDataOffset; // raw
  } else if (g_fh.compType == 2) {
    g_compBase = g_fh.chanDataOffset;
    bool perFrame = (g_compCount == g_fh.frameCount && g_fh.channelCount>0);
    if (perFrame){
      for (uint32_t i=0;i<g_compCount;++i){
        if (g_cblocks[i].uSize != g_fh.channelCount){ perFrame=false; break; }
      }
    }
#if defined(MZ_OK) || defined(Z_OK)
    g_compPerFrame = perFrame;
    if (!g_compPerFrame) { why="zlib block!=frame (not yet supported)"; return false; }
#else
    (void)perFrame;
    why="zlib not available in this toolchain";
    return false;
#endif
  } else {
    why="zstd unsupported";
    return false;
  }

  // Allocate frame buffer
  if (g_fh.channelCount==0){ why="zero chans"; return false; }
  g_frameBuf = (uint8_t*)malloc(g_fh.channelCount);
  if (!g_frameBuf){ why="oom frame"; return false; }

  g_currentPath = path;
  g_frameIndex  = 0;
  g_lastTickMs  = millis();
  g_playing     = true;

  Serial.printf("[FSEQ] %s frames=%lu chans=%lu step=%ums comp=%u blocks=%u sparse=%u CDO=0x%04x\n",
    path.c_str(), (unsigned long)g_fh.frameCount, (unsigned long)g_fh.channelCount,
    g_fh.stepTimeMs, g_fh.compType, (unsigned)g_compCount, (unsigned)g_fh.sparseCnt, g_fh.chanDataOffset);

#if !(defined(MZ_OK) || defined(Z_OK))
  if (g_fh.compType==2) {
    Serial.println("[FSEQ] NOTE: zlib file detected but no zlib/miniz headers found. Please export uncompressed or add zlib/miniz.");
  }
#endif
  return true;
}

static bool loadFrame(uint32_t idx){
  if (!g_fseq || !g_fh.frameCount) return false;
  idx %= g_fh.frameCount;

  if (g_fh.compType == 0){
    const uint64_t base = (uint64_t)g_fh.chanDataOffset + (uint64_t)idx * (uint64_t)g_fh.channelCount;
    if (!g_fseq.seek(base, SeekSet)) return false;
    return g_fseq.read(g_frameBuf, g_fh.channelCount) == g_fh.channelCount;
  }

#if defined(MZ_OK) || defined(Z_OK)
  if (g_fh.compType == 2 && g_compPerFrame){
    // offset into compressed stream
    uint64_t offs = g_compBase;
    for (uint32_t i=0;i<idx;++i) offs += g_cblocks[i].cSize;
    if (!g_fseq.seek(offs, SeekSet)) return false;
    uint32_t clen = g_cblocks[idx].cSize;
    if (clen == 0 || clen > 8*1024*1024) return false; // sanity
    uint8_t* ctmp = (uint8_t*)malloc(clen);
    if (!ctmp) return false;
    size_t got = g_fseq.read(ctmp, clen);
    bool ok = false;
    if (got == clen) ok = zlib_decompress(ctmp, clen, g_frameBuf, g_fh.channelCount);
    free(ctmp);
    return ok;
  }
#endif
  return false;
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

  // Use channels-per-spoke from the FSEQ frame size, not a fixed LED count.
  const uint32_t fallbackPixels = g_pixelsPerArm ? g_pixelsPerArm : DEFAULT_PIXELS_PER_ARM;
  const uint32_t chPerSpoke =
    (spokes > 0 && g_fh.channelCount) ? (g_fh.channelCount / spokes) : (fallbackPixels * 3u);

  const uint32_t startChBase = (g_startChArm1 > 0 ? g_startChArm1 - 1 : 0);
  const uint32_t indexOffset = (spokes > 0) ? (g_indexPosition % spokes) : 0;

  // ---- QUADRANT MAPPING: start spoke + stride (spokes/arms) ----
  const int startIdx0 = spoke1BasedToIdx0(START_SPOKE_1BASED, (int)spokes);

  for (uint8_t arm=0; arm<arms; ++arm){
    if (!strips[arm]) continue;

    // Map each arm to its spoke using quadrant stride, then add rotary index offset
    uint32_t armBaseSpoke = (uint32_t)armSpokeIdx0((int)arm, startIdx0, (int)spokes, (int)arms);
    armBaseSpoke = (armBaseSpoke + indexOffset) % spokes;

    const uint32_t baseChAbsR = startChBase + armBaseSpoke * chPerSpoke;
    const uint32_t pixelCount = strips[arm]->numPixels();

    for (uint32_t i=0; i<pixelCount; ++i){
      const uint32_t absR = baseChAbsR + i * blockStride;
      const int64_t  idxR = sparseTranslate(absR);
      if (idxR < 0 || (idxR+2) >= (int64_t)g_fh.channelCount) {
        strips[arm]->setPixelColor(i,0,0,0); continue;
      }
      uint8_t R,G,B; mapChannels(&g_frameBuf[idxR], R,G,B);
      strips[arm]->setPixelColor(i, R,G,B);
    }
    strips[arm]->show();
  }
  return true;
}


/* -------------------- Web: Files page + ops -------------------- */
static void handleFiles();     // list & UI (with upload form)
static void handleDownload();  // /dl?path=...
static void handlePlayLink();  // /play?path=...&back=...
static void handleDelete();    // /rm?path=...&back=...
static void handleMkdir();     // /mkdir?path=/current&name=New
static void handleRename();    // /ren?path=/old&to=NewName
static void handleUploadData();
static void handleUploadDone();

// Upload state
File   g_uploadFile;
String g_uploadFilename;   // full SD path of the file being written
size_t g_uploadBytes = 0;

static void handleFiles() {
  String path = server.hasArg("path") ? server.arg("path") : "/";
  if (!path.length() || path[0] != '/') path = "/" + path;

  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
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
      String base = name;
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);
      String baseEsc = htmlEscape(base);
      html += WebPages::filesDirectoryRow(esc, enc, esc, baseEsc, backParam);
    } else {
      uint64_t sz = ent.size();
      String base = name;
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);
      String baseEsc = htmlEscape(base);
      html += WebPages::filesFileRow(esc, enc, sz, esc, baseEsc, backParam);
    }
    ent.close();
  }
  dir.close();

  html += WebPages::filesPageFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleDownload() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f || f.isDirectory()) { if (f) f.close(); server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + htmlEscape(path.substring(path.lastIndexOf('/')+1)) + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

static void handlePlayLink() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;
  if (!SD_MMC.exists(path) || !isFseqName(path)) {
    server.sendHeader("Location", back); server.send(302, "text/plain", "Not a .fseq or missing"); return;
  }
  String why;
  openFseq(path, why);
  server.sendHeader("Location", back);
  server.send(302, "text/plain", "OK");
}

static void handleDelete() {
  if (!server.hasArg("path")) { server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  String back = server.hasArg("back") ? server.arg("back") : "/files?path=/";
  if (!path.startsWith("/")) path = "/" + path;

  File f = SD_MMC.open(path);
  bool ok=false;
  if (f) {
    if (f.isDirectory()) {
      f.close();
      ok = SD_MMC.rmdir(path);
    } else {
      f.close();
      ok = SD_MMC.remove(path);
    }
  }
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
  bool ok = SD_MMC.mkdir(base + name);
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
  bool ok = SD_MMC.rename(p, dst);
  server.sendHeader("Location", back);
  server.send(ok?302:500, "text/plain", ok?"Renamed":"Rename failed");
}

static void handleUploadData() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_uploadBytes = 0;

    // Destination dir & back target (optional)
    String dir = server.hasArg("dir") ? server.arg("dir") : "/";
    if (dir.indexOf("..") >= 0) dir = "/";            // basic safety
    if (!dir.startsWith("/")) dir = "/" + dir;

    // Filename (basename only)
    g_uploadFilename = up.filename;
    if (!g_uploadFilename.length()) g_uploadFilename = "upload.fseq";
    int slash = g_uploadFilename.lastIndexOf('/');
    if (slash >= 0) g_uploadFilename = g_uploadFilename.substring(slash + 1);

    // Compose full path inside selected dir
    g_uploadFilename = joinPath(dir, g_uploadFilename);

    if (!isFseqName(g_uploadFilename)) {
      Serial.printf("[UPLOAD] Rejected non-.fseq: %s\n", g_uploadFilename.c_str());
    } else {
      // Ensure dir exists
      File d = SD_MMC.open(dir);
      bool okdir = d && d.isDirectory();
      if (d) d.close();
      if (!okdir) {
        Serial.printf("[UPLOAD] Target dir missing: %s\n", dir.c_str());
      } else {
        if (SD_MMC.exists(g_uploadFilename)) SD_MMC.remove(g_uploadFilename);
        g_uploadFile = SD_MMC.open(g_uploadFilename, FILE_WRITE);
        Serial.printf("[UPLOAD] START %s\n", g_uploadFilename.c_str());
      }
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_uploadFile) { g_uploadFile.write(up.buf, up.currentSize); g_uploadBytes += up.currentSize; }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_uploadFile) { g_uploadFile.close(); Serial.printf("[UPLOAD] DONE %s (%u bytes)\n", g_uploadFilename.c_str(), (unsigned)g_uploadBytes); }
    else { Serial.println("[UPLOAD] Aborted/invalid file"); }
  }
}

static void handleUploadDone() {
  String back = server.hasArg("back") ? server.arg("back") : "/";
  if (!isFseqName(g_uploadFilename)) {
    server.send(415, "text/html",
      "<meta http-equiv='refresh' content='2;url=" + back + "'>"
      "<body style='font-family:system-ui'><p>Upload rejected. Only <b>.fseq</b> files are allowed.</p><p>Returning…</p></body>");
    return;
  }
  if (!SD_MMC.exists(g_uploadFilename)) {
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
        optionsHtml += "'>";
        optionsHtml += name;
        optionsHtml += "</option>";
      }
    }
    ent.close();
  }
  dir.close();
}

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
    +",\"stride\":\""+String(g_strideMode==STRIDE_SPOKE?"spoke":"led")+"\""
    +"}";
  server.send(200,"application/json",json);
}

static void handleRoot() {
  String options; listFseqInDir("/", options);
  String cur = g_currentPath.length() ? g_currentPath : "(none)";
  String curEsc = htmlEscape(cur);
  String apIp = AP_IP.toString();
  String html = WebPages::rootPage(String(statusClass()), String(statusText()), curEsc, options,
                                   String(AP_SSID), apIp, String("pov.local"),
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
  prefs.putUChar("brightness", g_brightnessPercent); // persisted in NVS
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
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
  server.send(200, "application/json", String("{\"fps\":") + g_fps + "}");
}

// POST /mapcfg?start=1&spokes=40&arms=4&pixels=144&stride=spoke|led
static void handleMapCfg(){
  bool needRebuild = false;

  if (server.hasArg("start"))  { uint32_t v=strtoul(server.arg("start").c_str(),nullptr,10); g_startChArm1=(v<1)?1:v; prefs.putULong("startch", g_startChArm1); }
  if (server.hasArg("spokes")) { int v=server.arg("spokes").toInt(); if (v<1) v=1; g_spokesTotal=(uint16_t)v; prefs.putUShort("spokes", g_spokesTotal); }
  if (server.hasArg("arms"))   { int v=server.arg("arms").toInt(); uint8_t nv=clampArmCount(v); if (nv!=g_armCount){ g_armCount=nv; prefs.putUChar("arms", g_armCount); needRebuild=true; } }
  if (server.hasArg("pixels")) { int v=server.arg("pixels").toInt(); uint16_t np=clampPixelsPerArm(v); if (np!=g_pixelsPerArm){ g_pixelsPerArm=np; prefs.putUShort("pixels", g_pixelsPerArm); needRebuild=true; } }
  if (server.hasArg("stride")) {
    String s = server.arg("stride"); s.toLowerCase();
    g_strideMode = (s=="led") ? STRIDE_LED : STRIDE_SPOKE;
    prefs.putUChar("stride", (uint8_t)g_strideMode);
  }
  if (needRebuild) rebuildStrips();
  server.send(200,"application/json",
    String("{\"start\":")+g_startChArm1+",\"spokes\":"+g_spokesTotal+",\"arms\":"+(int)g_armCount+
    ",\"pixels\":"+g_pixelsPerArm+",\"stride\":\""+(g_strideMode==STRIDE_SPOKE?"spoke":"led")+"\"}");
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
  if (SD_MMC.cardType()!=CARD_NONE) ok=true;
  else ok = mountSdmmc();
  if (!ok) { server.send(500,"text/plain","SD not present"); return; }
  if (!g_currentPath.length()) { server.send(200,"text/plain","SD OK; no file"); return; }
  String why;
  if (openFseq(g_currentPath, why)) server.send(200,"text/plain","SD OK; file reopened");
  else server.send(500,"text/plain", String("reopen fail: ")+why);
}

/* -------------------- Server, Setup, Loop -------------------- */
static void startWifiAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  if (MDNS.begin("pov")) MDNS.addService("http","tcp",80);

  // Control + status
  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/status",  HTTP_GET,  handleStatus);

  // Playback & settings
  server.on("/b",       HTTP_POST, handleB);
  server.on("/start",   HTTP_GET,  handleStart);
  server.on("/stop",    HTTP_POST, handleStop);
  server.on("/speed",   HTTP_POST, handleSpeed);
  server.on("/mapcfg",  HTTP_POST, handleMapCfg);

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

  // Upload (works for both control & files pages)
  server.on("/upload",  HTTP_POST, handleUploadDone, handleUploadData);

  server.onNotFound([](){
    server.send(404, "text/plain", String("404 Not Found: ") + server.uri());
  });

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
    // display as 1-based so it matches xLights labels
    Serial.printf("Arm %d → spoke %d\n", k+1, s0 + 1);
  }
  // Expect: 1, 11, 20, 30 when START_SPOKE_1BASED=1 and gLabelMode=FLOOR_TO_BOUNDARY

  // Restore settings
  prefs.begin("display", false);
  g_brightnessPercent = prefs.getUChar("brightness", 25);
  g_brightness        = (uint8_t)((255 * g_brightnessPercent) / 100);
  g_fps               = prefs.getUShort("fps", 40); if (!g_fps) g_fps=40;
  g_framePeriodMs     = 1000UL / g_fps;

  g_startChArm1       = prefs.getULong("startch", 1);
  g_spokesTotal       = prefs.getUShort("spokes", 40);
  if (!g_spokesTotal) g_spokesTotal = 1;
  g_armCount          = clampArmCount(prefs.getUChar("arms", MAX_ARMS));
  g_pixelsPerArm      = clampPixelsPerArm(prefs.getUShort("pixels", DEFAULT_PIXELS_PER_ARM));
  g_strideMode        = (StrideMode)prefs.getUChar("stride", (uint8_t)STRIDE_SPOKE);

  Serial.printf("[BRIGHTNESS] %u%% (%u)\n", g_brightnessPercent, g_brightness);
  Serial.printf("[PLAY] FPS=%u  period=%lums\n", g_fps, (unsigned long)g_framePeriodMs);
  Serial.printf("[MAP] startCh(Arm1)=%lu spokes=%u arms=%u pixels/arm=%u stride=%s\n",
                (unsigned long)g_startChArm1, g_spokesTotal, (unsigned)activeArmCount(),
                (unsigned)g_pixelsPerArm, (g_strideMode==STRIDE_SPOKE?"SPOKE":"LED"));

  startWifiAP();

  if (!cardPresent()) {
    Serial.printf("[SD] No card (CD HIGH on GPIO%d); UI still available.\n", PIN_SD_CD);
  }
  if (!mountSdmmc()) {
    Serial.println("[SD] Mount failed; UI still available for diagnostics.");
  } else {
    uint8_t type = SD_MMC.cardType();
    Serial.printf("[SD] Type=%u  Size=%llu MB\n",
                  (unsigned)type, (unsigned long long)(SD_MMC.cardSize() / (1024ULL*1024ULL)));
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
    Serial.println("[PLAY] frame read failed — attempting SD reinit + reopen");
    if (SD_MMC.cardType()==CARD_NONE) mountSdmmc();
    if (g_currentPath.length()){
      String why; openFseq(g_currentPath, why);
    }
    return;
  }
  g_frameIndex = (g_frameIndex + 1) % (g_fh.frameCount ? g_fh.frameCount : 1);
}
