#include "app.hpp"
#include "playback_timing.hpp"
#include "strip_speed.hpp"
#include "sd_recovery.hpp"
#include "pov_build_info.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <map>
#include <sys/stat.h>
#include <unistd.h>
#include <utime.h>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_core_dump.h"
#include "esp_partition.h"

extern const unsigned char pageStart[] asm("_binary_index_html_start");
extern const unsigned char pageEnd[] asm("_binary_index_html_end");
extern const unsigned char wifiPageStart[] asm("_binary_wifi_html_start");
extern const unsigned char wifiPageEnd[] asm("_binary_wifi_html_end");
extern const unsigned char ledPageStart[] asm("_binary_leds_html_start");
extern const unsigned char ledPageEnd[] asm("_binary_leds_html_end");
extern const unsigned char speedPageStart[] asm("_binary_speed_html_start");
extern const unsigned char speedPageEnd[] asm("_binary_speed_html_end");
extern const unsigned char sdPageStart[] asm("_binary_sd_html_start");
extern const unsigned char sdPageEnd[] asm("_binary_sd_html_end");
extern const unsigned char scopePageStart[] asm("_binary_scope_html_start");
extern const unsigned char scopePageEnd[] asm("_binary_scope_html_end");
extern const unsigned char recentScriptStart[] asm("_binary_recent_js_start");
extern const unsigned char recentScriptEnd[] asm("_binary_recent_js_end");
extern const unsigned char dutyScriptStart[] asm("_binary_duty_js_start");
extern const unsigned char dutyScriptEnd[] asm("_binary_duty_js_end");

namespace pov {
namespace {
using Params = std::map<std::string, std::string>;
esp_err_t sendJson(httpd_req_t* req, cJSON* object, const char* status = "200 OK") {
    char* text = cJSON_PrintUnformatted(object);
    cJSON_Delete(object);
    if (!text) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON allocation failed");
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}
esp_err_t error(httpd_req_t* req, const std::string& reason, const char* status = "400 Bad Request") {
    cJSON* object = cJSON_CreateObject();
    cJSON_AddStringToObject(object, "error", reason.c_str());
    return sendJson(req, object, status);
}
esp_err_t success(httpd_req_t* req) {
    cJSON* object = cJSON_CreateObject(); cJSON_AddBoolToObject(object, "ok", true); return sendJson(req, object);
}
bool decode(const std::string& input, std::string& output) {
    output.clear();
    auto hex = [](char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1; };
    for (size_t i = 0; i < input.size(); ++i) {
        unsigned char c = input[i];
        if (c == '%') {
            if (i + 2 >= input.size()) return false;
            const int a = hex(input[i + 1]), b = hex(input[i + 2]);
            if (a < 0 || b < 0) return false;
            c = a * 16 + b; i += 2;
        } else if (c == '+') c = ' ';
        if (!c || c == '\r' || c == '\n') return false;
        output += static_cast<char>(c);
    }
    return true;
}
bool parseParams(const std::string& text, Params& params) {
    size_t at = 0;
    while (at < text.size()) {
        size_t end = text.find('&', at);
        if (end == std::string::npos) end = text.size();
        auto entry = text.substr(at, end - at);
        auto equals = entry.find('=');
        std::string key, value;
        if (!decode(entry.substr(0, equals), key) || !decode(equals == std::string::npos ? "" : entry.substr(equals + 1), value)) return false;
        params[key] = value;
        at = end + 1;
    }
    return true;
}
bool number(const Params& p, const char* key, double low, double high, double& value, bool integer = true) {
    auto item = p.find(key);
    if (item == p.end() || item->second.empty()) return false;
    char* end;
    errno = 0;
    value = strtod(item->second.c_str(), &end);
    return !errno && *end == '\0' && std::isfinite(value) && value >= low && value <= high && (!integer || std::floor(value) == value);
}
bool boolean(const Params& p, const char* key, bool& value) {
    auto item = p.find(key);
    if (item == p.end()) return false;
    if (item->second == "1" || item->second == "true") value = true;
    else if (item->second == "0" || item->second == "false") value = false;
    else return false;
    return true;
}
std::string value(const Params& p, const char* key, const char* fallback = "") {
    const auto found = p.find(key); return found == p.end() ? fallback : found->second;
}
cJSON* outputJson() {
    cJSON* j = cJSON_CreateObject();
    const auto& stats = output.stats;
    cJSON_AddStringToObject(j, "driver", output.dmaEnabled() ? "native-idf-lcd-gdma" : "native-idf-shared-clock-gpio");
    cJSON_AddBoolToObject(j, "dma", output.dmaEnabled());
    cJSON_AddBoolToObject(j, "ready", output.ready());
    cJSON_AddStringToObject(j, "lastError", esp_err_to_name(output.lastError()));
    cJSON_AddStringToObject(j, "protocol", config.ledProtocol == 1 ? "APA102" : "SK9822");
    cJSON_AddNumberToObject(j, "brightnessPercent", config.brightness);
    cJSON_AddBoolToObject(j, "centerDimming", config.centerDimming);
    cJSON_AddNumberToObject(j, "centerBrightnessPercent", config.centerDimming ?
        std::min(config.centerBrightness, config.brightness) : config.brightness);
    cJSON_AddNumberToObject(j, "clockPin", BoardPins::Clock);
    cJSON_AddNumberToObject(j, "motorSpeedPwmPin", BoardPins::MotorSpeedPwm);
    cJSON_AddNumberToObject(j, "clockLevel", gpio_get_level(static_cast<gpio_num_t>(BoardPins::Clock)));
    cJSON* signal = cJSON_AddObjectToObject(j, "signalCheck");
    cJSON_AddBoolToObject(signal, "running", runtime.mode == Mode::SignalCheck);
    cJSON_AddNumberToObject(signal, "pin", runtime.signalPin);
    cJSON_AddNumberToObject(signal, "pattern", runtime.signalPattern);
    cJSON_AddBoolToObject(signal, "high", runtime.signalHigh);
    cJSON_AddNumberToObject(j, "pixelsPerStrip", config.pixels);
    cJSON_AddNumberToObject(j, "bitsPerStrip", stats.bits);
    cJSON_AddNumberToObject(j, "lastTransmit_us", stats.lastUs);
    cJSON_AddNumberToObject(j, "lastPack_us", stats.lastPackUs);
    cJSON_AddNumberToObject(j, "lastSubmitToDone_us", stats.lastCompletionUs);
    cJSON_AddNumberToObject(j, "minTransmit_us", stats.minUs);
    cJSON_AddNumberToObject(j, "maxTransmit_us", stats.maxUs);
    cJSON_AddNumberToObject(j, "meanTransmit_us", stats.count ? double(stats.totalUs) / stats.count : 0);
    cJSON_AddNumberToObject(j, "transmissions", stats.count);
    cJSON_AddNumberToObject(j, "colorTransmissions", stats.colorCount);
    cJSON_AddNumberToObject(j, "blackTransmissions", stats.blackCount);
    cJSON_AddNumberToObject(j, "effectiveMbps", stats.lastUs ? double(stats.bits) / stats.lastUs : 0);
    cJSON_AddNumberToObject(j, "psramBytes", esp_psram_get_size());
    cJSON_AddNumberToObject(j, "freePsramBytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(j, "hallPin", BoardPins::Hall);
    cJSON_AddNumberToObject(j, "sdD0Pin", BoardPins::SdD0);
    cJSON* arms = cJSON_AddArrayToObject(j, "arms");
    for (unsigned arm = 0; arm < MaxArms; ++arm) {
        cJSON* entry = cJSON_CreateObject();
        cJSON_AddNumberToObject(entry, "arm", arm + 1);
        cJSON_AddNumberToObject(entry, "connector", BoardPins::ActiveConnectors[arm]);
        cJSON_AddNumberToObject(entry, "angleDegrees", armAngleDegrees(arm, config.armClockwise, config.rotationClockwise));
        cJSON_AddBoolToObject(entry, "inputAtHub", BoardPins::ArmInputAtHub[arm]);
        cJSON_AddBoolToObject(entry, "imageReversed", BoardPins::ArmReverse[arm] !=
            (runtime.mode == Mode::Playback && runtime.sequenceTipFirst));
        cJSON_AddNumberToObject(entry, "dataPin", BoardPins::ArmData[arm]);
        cJSON_AddBoolToObject(entry, "active", arm < config.arms);
        cJSON_AddNumberToObject(entry, "level", gpio_get_level(static_cast<gpio_num_t>(BoardPins::ArmData[arm])));
        cJSON_AddItemToArray(arms, entry);
    }
    auto* orderTest = cJSON_AddObjectToObject(j, "armOrderTest");
    const bool orderRunning = runtime.mode == Mode::ArmOrder;
    const int64_t orderElapsed = orderRunning ? esp_timer_get_time() - runtime.diagnosticStarted : 0;
    cJSON_AddBoolToObject(orderTest, "running", orderRunning);
    cJSON_AddNumberToObject(orderTest, "activeArm", orderRunning ? armOrderTestArm(orderElapsed, config.arms) + 1 : 0);
    cJSON_AddNumberToObject(orderTest, "remainingSeconds", orderRunning
        ? std::max<int64_t>(0, ArmOrderTestDurationUs - orderElapsed + 999999) / 1000000 : 0);
    return j;
}
cJSON* hallFilterJson() {
    const auto hall = hallSnapshot();
    auto* j = cJSON_CreateObject();
    cJSON_AddNumberToObject(j, "maxAcceptedRpm", MaximumHallRpm);
    cJSON_AddNumberToObject(j, "minimumPeriod_us", MinimumHallPeriodUs);
    cJSON_AddNumberToObject(j, "averageWindow", HallAverageSamples);
    cJSON_AddNumberToObject(j, "averageSamples", hall.samples);
    cJSON_AddNumberToObject(j, "averagePeriod_us", hall.period);
    cJSON_AddNumberToObject(j, "lastAcceptedPeriod_us", hall.lastPeriod);
    cJSON_AddNumberToObject(j, "acceptedPulses", hall.count);
    cJSON_AddNumberToObject(j, "rejectedPulses", hall.rejected);
    cJSON_AddNumberToObject(j, "lastRejectedInterval_us", hall.lastRejectedInterval);
    return j;
}
cJSON* flashProofJson() {
    auto* j = cJSON_CreateObject();
    const bool running = flashProofRunning();
    const double wireUs = SharedClockProtocol::frameBytes(config.pixels,
        static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8.0 * 1000000 / PlaybackDmaClockHz;
    const auto hall = hallSnapshot();
    const double slotUs = hall.period > 0 ? double(hall.period) / displaySpokes() : 0;
    cJSON_AddBoolToObject(j, "running", running);
    cJSON_AddStringToObject(j, "state", runtime.flashProofState.c_str());
    cJSON_AddNumberToObject(j, "remainingSeconds", running ?
        std::max<int64_t>(0, runtime.flashProofUntil - esp_timer_get_time() + 999999) / 1000000 : 0);
    cJSON_AddNumberToObject(j, "colorFrames", runtime.flashProofColorFrames);
    cJSON_AddNumberToObject(j, "nominalPulse_us", wireUs * runtime.flashProofColorFrames);
    cJSON_AddNumberToObject(j, "brightnessPercent", config.brightness);
    cJSON_AddNumberToObject(j, "burstWire_us", wireUs * (runtime.flashProofColorFrames + 1));
    cJSON_AddNumberToObject(j, "nominalDutyPercent", rpm() > 0 && slotUs > 0 ? wireUs * runtime.flashProofColorFrames * 100 / slotUs : 0);
    cJSON_AddNumberToObject(j, "bursts", runtime.flashProofBursts);
    cJSON_AddNumberToObject(j, "latePreparedPulses", runtime.flashProofSkipped);
    return j;
}
cJSON* dutyControlJson() {
    const auto calculated = dutyCalculation();
    const bool active = config.autoDuty && usesDisplayDuty(runtime.mode) && !flashProofRunning();
    cJSON* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "active", active);
    cJSON_AddBoolToObject(j, "available", calculated.available);
    cJSON_AddBoolToObject(j, "feasible", calculated.feasible);
    cJSON_AddBoolToObject(j, "continuous", calculated.continuous);
    cJSON_AddNumberToObject(j, "calculatedDuty", calculated.percent);
    cJSON_AddNumberToObject(j, "effectiveDuty", config.autoDuty ? calculated.percent : config.duty);
    cJSON_AddNumberToObject(j, "spokeDuration_us", calculated.spokeUs);
    cJSON_AddNumberToObject(j, "transferBudget_us", calculated.transferUs);
    cJSON_AddNumberToObject(j, "blankBudget_us", calculated.blankUs);
    return j;
}
cJSON* fileSettingsJson() {
    const auto derived = fileSettings();
    cJSON* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "loaded", !runtime.filePath.empty());
    cJSON_AddStringToObject(j, "path", runtime.filePath.c_str());
    cJSON_AddNumberToObject(j, "stepTimeMs", runtime.fileHeader.stepMs);
    cJSON_AddNumberToObject(j, "channelCount", runtime.fileHeader.channels);
    cJSON_AddNumberToObject(j, "inferredSpokes", derived.spokes);
    cJSON_AddBoolToObject(j, "fpsFromFile", config.autoFseq && derived.frameUs);
    cJSON_AddBoolToObject(j, "spokesFromFile", config.autoFseq && derived.spokes);
    cJSON_AddNumberToObject(j, "effectiveFps", 1000000.0 / sequenceFrameUs());
    cJSON_AddNumberToObject(j, "framePeriod_us", sequenceFrameUs());
    cJSON_AddNumberToObject(j, "effectiveSpokes", displaySpokes());
    cJSON_AddStringToObject(j, "timingNote", derived.timingNote);
    cJSON_AddStringToObject(j, "layoutNote", derived.layoutNote);
    return j;
}
esp_err_t fileList(httpd_req_t* req, const Params& p) {
    sequence.quiescent();
    if (sequence.loading()) return error(req, "SD is busy; retry shortly", "503 Service Unavailable");
    std::string path = value(p, "path", "/"), full;
    if (!sd.ready || !sdPath(path, full)) return error(req, "SD unavailable or invalid path");
    DIR* dir = opendir(full.c_str());
    if (!dir) {
        if (sdIoError(errno)) requestSdRecovery("SD directory read failed");
        return error(req, "Cannot open directory", "404 Not Found");
    }
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "path", path.c_str());
    cJSON* entries = cJSON_AddArrayToObject(j, "files");
    unsigned count = 0;
    while (dirent* entry = readdir(dir)) {
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        std::string child = path + (path.back() == '/' ? "" : "/") + entry->d_name;
        std::string native;
        if (!sdPath(child, native)) continue;
        struct stat info;
        if (stat(native.c_str(), &info)) continue;
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", entry->d_name);
        cJSON_AddStringToObject(item, "path", child.c_str());
        cJSON_AddBoolToObject(item, "directory", S_ISDIR(info.st_mode));
        cJSON_AddNumberToObject(item, "size", info.st_size);
        if (info.st_mtime > 0) cJSON_AddNumberToObject(item, "modified", static_cast<double>(info.st_mtime));
        else cJSON_AddNullToObject(item, "modified");
        cJSON_AddItemToArray(entries, item);
        if (++count == 512) { cJSON_AddBoolToObject(j, "truncated", true); break; }
    }
    closedir(dir);
    return sendJson(req, j);
}
esp_err_t download(httpd_req_t* req, const Params& p) {
    std::string full;
    FILE* file;
    { Lock lock;
      if (!sd.ready || !sdPath(value(p, "path"), full)) return error(req, "Invalid download path");
      stopPlayback(); sequence.close();
      if (!sequence.quiescent()) return error(req, "SD read is still finishing; retry shortly", "503 Service Unavailable");
      file = fopen(full.c_str(), "rb");
      if (file) {
          stopPlayback(); sequence.close(); runtime.maintenance = true;
      }
    }
    if (!file) {
        const int code = errno;
        if (sdIoError(code)) { Lock lock; requestSdRecovery("SD download open failed"); }
        return error(req, "File not found", "404 Not Found");
    }
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment");
    char buffer[4096];
    esp_err_t result = ESP_OK;
    while (true) {
        size_t count;
        { Lock lock; count = fread(buffer, 1, sizeof(buffer), file); }
        if (!count) break;
        result = httpd_resp_send_chunk(req, buffer, count);
        if (result != ESP_OK) break;
    }
    {
        Lock lock;
        const bool readFailed = ferror(file) != 0;
        if (readFailed) result = ESP_FAIL;
        fclose(file);
        runtime.maintenance = false;
        runtime.lastActivity = esp_timer_get_time();
        if (readFailed) requestSdRecovery("SD download read failed");
    }
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, nullptr, 0);
    return result;
}
esp_err_t upload(httpd_req_t* req, const std::string& route, const Params& p) {
    if (sdToolRunning()) return error(req, "Wait for the SD operation to finish", "409 Conflict");
    char type[96] = {};
    httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type));
    if (strstr(type, "multipart/")) return error(req, "Use the native web page or send the file as a raw request body");
    if (!req->content_len) return error(req, "Empty upload");
    const bool direct = route == "/ota";
    const bool firmware = direct || route == "/fw/upload";
    double modified = 0;
    if (!direct && p.count("modified") && !number(p, "modified", 315532800.0, 4354819199.0, modified))
        return error(req, "Invalid file modified date");
    std::string relative = firmware ? "/firmware.bin" : value(p, "path");
    std::string full;
    const esp_partition_t* target = direct ? esp_ota_get_next_update_partition(nullptr) : nullptr;
    if (direct && (!target || req->content_len > target->size)) return error(req, "Firmware does not fit OTA partition");
    if (!direct && (!sdPath(relative, full) || relative == "/")) return error(req, "Invalid upload path");
    if (firmware && req->content_len > 0x650000) return error(req, "Firmware is too large");
    esp_ota_handle_t ota = 0;
    FILE* file = nullptr;
    { Lock lock;
      if (sdToolRunning()) return error(req, "Wait for the SD operation to finish", "409 Conflict");
      stopPlayback(); sequence.close();
      if (!sequence.quiescent()) return error(req, "SD read is still finishing; retry shortly", "503 Service Unavailable");
      if (direct) {
          const auto err = esp_ota_begin(target, req->content_len, &ota);
          if (err != ESP_OK) return error(req, esp_err_to_name(err), "500 Internal Server Error");
      } else {
          if (!sd.ready) return error(req, "SD is not mounted");
          file = fopen((full + ".upload").c_str(), "wb");
          if (!file) {
              if (sdIoError(errno)) requestSdRecovery("SD upload file creation failed");
              return error(req, "Cannot create upload file");
          }
      }
      runtime.maintenance = true;
    }
    char buffer[4096];
    size_t remaining = req->content_len;
    unsigned timeouts = 0;
    bool sdFailed = false;
    esp_err_t result = ESP_OK;
    while (remaining) {
        const int count = httpd_req_recv(req, buffer, std::min(remaining, sizeof(buffer)));
        if (count == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 3) continue;
        if (count <= 0) { result = ESP_FAIL; break; }
        timeouts = 0;
        if (direct) result = esp_ota_write(ota, buffer, count);
        else { Lock lock; if (fwrite(buffer, 1, count, file) != static_cast<size_t>(count)) { sdFailed |= sdIoError(errno); result = ESP_FAIL; } }
        if (result != ESP_OK) break;
        remaining -= count;
    }
    if (direct) {
        if (result == ESP_OK) result = esp_ota_end(ota);
        else esp_ota_abort(ota);
        if (result == ESP_OK) result = esp_ota_set_boot_partition(target);
    } else {
        Lock lock;
        if (fclose(file) != 0) { sdFailed |= sdIoError(errno); result = ESP_FAIL; }
        if (result == ESP_OK) {
            remove(full.c_str());
            if (rename((full + ".upload").c_str(), full.c_str()) != 0) { sdFailed |= sdIoError(errno); result = ESP_FAIL; }
            if (result == ESP_OK && modified) {
                const struct utimbuf times = {static_cast<time_t>(modified), static_cast<time_t>(modified)};
                if (utime(full.c_str(), &times) != 0) log("SD upload: could not preserve file modified date");
            }
        }
        if (result != ESP_OK) remove((full + ".upload").c_str());
    }
    { Lock lock; runtime.maintenance = false; runtime.lastActivity = esp_timer_get_time();
      if (sdFailed) requestSdRecovery("SD upload write failed"); }
    if (result != ESP_OK) return error(req, std::string("Upload failed: ") + esp_err_to_name(result), "500 Internal Server Error");
    log("Upload complete: %s (%u bytes)", direct ? "OTA partition" : relative.c_str(), req->content_len);
    const auto sent = success(req);
    if (direct) restartSoon();
    return sent;
}

esp_err_t get(httpd_req_t* req, const std::string& route, const Params& p) {
    if (route == "/duty.js") {
        httpd_resp_set_type(req, "application/javascript; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(dutyScriptStart), dutyScriptEnd - dutyScriptStart - 1);
    }
    if (sdToolRunning() && (route == "/api/files" || route == "/dl" || route == "/start" || route == "/play"))
        return error(req, "Wait for the SD operation to finish", "409 Conflict");
    if (route == "/sd/tools" || route == "/sd/scope/data") {
        cJSON* j;
        { Lock lock; j = route == "/sd/tools" ? sdToolJson() : sdScopeJson(); }
        return sendJson(req, j);
    }
    if (route == "/sd/scope") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(scopePageStart), scopePageEnd - scopePageStart - 1);
    }
    if (route == "/diag/crash") {
        auto* j = cJSON_CreateObject();
        size_t address = 0, bytes = 0;
        esp_err_t err = esp_core_dump_image_get(&address, &bytes);
        if (err == ESP_OK) err = esp_core_dump_image_check();
        cJSON_AddBoolToObject(j, "available", err == ESP_OK);
        cJSON_AddStringToObject(j, "status", esp_err_to_name(err));
        if (err == ESP_OK) {
            esp_core_dump_summary_t summary = {};
            err = esp_core_dump_get_summary(&summary);
            cJSON_AddNumberToObject(j, "bytes", bytes);
            cJSON_AddStringToObject(j, "summaryStatus", esp_err_to_name(err));
            if (err == ESP_OK) {
                cJSON_AddStringToObject(j, "task", summary.exc_task);
                cJSON_AddNumberToObject(j, "pc", summary.exc_pc);
                cJSON_AddNumberToObject(j, "cause", summary.ex_info.exc_cause);
                cJSON_AddNumberToObject(j, "address", summary.ex_info.exc_vaddr);
                cJSON_AddStringToObject(j, "elfSha256", reinterpret_cast<const char*>(summary.app_elf_sha256));
                cJSON_AddBoolToObject(j, "backtraceCorrupted", summary.exc_bt_info.corrupted);
                auto* trace = cJSON_AddArrayToObject(j, "backtrace");
                for (unsigned i = 0; i < std::min<unsigned>(summary.exc_bt_info.depth, 16); ++i)
                    cJSON_AddItemToArray(trace, cJSON_CreateNumber(summary.exc_bt_info.bt[i]));
            }
        }
        return sendJson(req, j);
    }
    if (route == "/diag/crash.bin") {
        size_t address = 0, bytes = 0;
        const auto* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
        if (!partition || esp_core_dump_image_get(&address, &bytes) != ESP_OK ||
            esp_core_dump_image_check() != ESP_OK || bytes > partition->size)
            return error(req, "No valid saved crash report", "404 Not Found");
        httpd_resp_set_type(req, "application/octet-stream");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=crash.bin");
        uint8_t buffer[1024];
        for (size_t at = 0; at < bytes; at += sizeof(buffer)) {
            const size_t count = std::min(sizeof(buffer), bytes - at);
            if (esp_partition_read(partition, at, buffer, count) != ESP_OK ||
                httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buffer), count) != ESP_OK)
                return ESP_FAIL;
        }
        return httpd_resp_send_chunk(req, nullptr, 0);
    }
    if (route == "/speed-test" || route == "/speed-test/" || route == "/speed.html") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(speedPageStart), speedPageEnd - speedPageStart - 1);
    }
    if (route == "/setup" || route == "/setup/" || route == "/setup.html") {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/leds#setup");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, nullptr, 0);
    }
    if (route == "/leds" || route == "/leds/" || route == "/leds.html") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(ledPageStart), ledPageEnd - ledPageStart - 1);
    }
    if (route == "/wifi" || route == "/wifi/" || route == "/wifi.html") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(wifiPageStart), wifiPageEnd - wifiPageStart - 1);
    }
    if (route == "/wifi/scan") return sendJson(req, wifiScanJson());
    if (route == "/sd" || route == "/sd/" || route == "/sd.html") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(sdPageStart), sdPageEnd - sdPageStart - 1);
    }
    if (route == "/files" || route == "/diagnostics" || route == "/updates" || route == "/ota" || route == "/logs") {
        const std::string target = "/sd#" + (route == "/ota" ? std::string("updates") : route.substr(1));
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", target.c_str());
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, nullptr, 0);
    }
    if (route == "/recent.js") {
        httpd_resp_set_type(req, "application/javascript; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(recentScriptStart), recentScriptEnd - recentScriptStart - 1);
    }
    if (route == "/" || route == "/index.html") {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, reinterpret_cast<const char*>(pageStart), pageEnd - pageStart - 1);
    }
    if (route == "/dl") return download(req, p);
    if (route == "/logs.txt") { const auto text = logText(); httpd_resp_set_type(req, "text/plain; charset=utf-8"); return httpd_resp_sendstr(req, text.c_str()); }
    if (route == "/diag/wifi") {
        cJSON* j = cJSON_CreateObject(); addNetworkStatus(j);
        return sendJson(req, j);
    }
    // Release the display/state mutex before a slow or disconnected client can
    // block the HTTP send. These are the frequently polled snapshots.
    if (route == "/status" || route == "/diag/dma" || route == "/diag/spi" || route == "/diag/strip-speed") {
        cJSON* j;
        { Lock lock; j = route == "/status" ? statusJson() : route == "/diag/dma" ? dmaTestJson() :
            route == "/diag/strip-speed" ? stripSpeedTestJson() : outputJson(); }
        return sendJson(req, j);
    }
    Lock lock;
    if (route == "/api/files") return fileList(req, p);
    if (route == "/diag/timing" || route == "/diag/blank" || route == "/diag/duty") {
        const auto hall = hallSnapshot();
        cJSON* j = cJSON_CreateObject();
        cJSON_AddNumberToObject(j, "targetFps", 1000000.0 / sequenceFrameUs());
        cJSON_AddNumberToObject(j, "frameCounter", runtime.framesRead);
        cJSON_AddNumberToObject(j, "targetPeriod_us", sequenceFrameUs());
        cJSON_AddNumberToObject(j, "lastPeriod_us", hall.period);
        cJSON_AddItemToObject(j, "hallFilter", hallFilterJson());
        cJSON_AddNumberToObject(j, "spokeDuration_us", double(hall.period) / displaySpokes());
        cJSON_AddNumberToObject(j, "spokes", displaySpokes());
        cJSON_AddItemToObject(j, "fileSettings", fileSettingsJson());
        cJSON_AddItemToObject(j, "flashProof", flashProofJson());
        const unsigned duty = config.autoDuty ? dutyCalculation().percent : config.duty;
        cJSON_AddNumberToObject(j, "dutyPercent", duty);
        cJSON_AddNumberToObject(j, "manualDutyPercent", config.duty);
        const double slotUs = double(hall.period) / displaySpokes();
        const double holdUs = flashProofRunning() ? SharedClockProtocol::frameBytes(config.pixels,
            static_cast<SharedClockProtocol::Protocol>(config.ledProtocol)) * 8.0 * 1000000 * runtime.flashProofColorFrames / PlaybackDmaClockHz : config.strobe && !config.autoDuty ?
            std::min(slotUs, hall.period * double(config.strobeWidth) / 360) : slotUs * duty / 100;
        cJSON_AddNumberToObject(j, "holdDuration_us", flashProofRunning() || duty ? holdUs : 0);
        cJSON_AddBoolToObject(j, "holdDurationIsNominal", flashProofRunning());
        cJSON_AddItemToObject(j, "dutyControl", dutyControlJson());
        cJSON_AddNumberToObject(j, "missedSpokes", runtime.missedSpokes);
        cJSON_AddNumberToObject(j, "lastFrameLoad_us", runtime.lastFrameLoadUs);
        cJSON_AddNumberToObject(j, "maxFrameLoad_us", runtime.maxFrameLoadUs);
        cJSON_AddNumberToObject(j, "frameLoadTimeout_us", FrameLoadTimeoutUs);
        cJSON_AddNumberToObject(j, "sdStalls", runtime.sdStalls);
        cJSON_AddNumberToObject(j, "skippedPaints", runtime.skippedPaints);
        cJSON_AddNumberToObject(j, "maxBlankStartLate_us", runtime.maxBlankStartLateUs);
        cJSON_AddNumberToObject(j, "lastPaintPrepare_us", runtime.lastPaintPrepareUs);
        cJSON_AddNumberToObject(j, "maxPaintPrepare_us", runtime.maxPaintPrepareUs);
        cJSON_AddBoolToObject(j, "timingLimited", runtime.timingLimited);
        cJSON_AddNumberToObject(j, "playbackClockHz", PlaybackDmaClockHz);
        cJSON_AddItemToObject(j, "output", outputJson());
        return sendJson(req, j);
    }
    if (route == "/fseq/header" || route == "/fseq/cblocks" || route == "/fseq/ranges") {
        cJSON* j = cJSON_CreateObject();
        const auto& h = sequence.header;
        cJSON_AddNumberToObject(j, "major", h.major); cJSON_AddNumberToObject(j, "minor", h.minor);
        cJSON_AddNumberToObject(j, "channelCount", h.channels); cJSON_AddNumberToObject(j, "frameCount", h.frames);
        cJSON_AddNumberToObject(j, "stepTimeMs", h.stepMs); cJSON_AddNumberToObject(j, "compression", h.compression);
        cJSON_AddNumberToObject(j, "dataOffset", h.dataOffset);
        cJSON* blocks = cJSON_AddArrayToObject(j, "blocks");
        for (const auto& b : sequence.blocks) {
            cJSON* entry = cJSON_CreateObject();
            cJSON_AddNumberToObject(entry, "firstFrame", b.firstFrame); cJSON_AddNumberToObject(entry, "compressedBytes", b.length);
            cJSON_AddItemToArray(blocks, entry);
        }
        cJSON* ranges = cJSON_AddArrayToObject(j, "ranges");
        for (const auto& r : sequence.ranges) {
            cJSON* entry = cJSON_CreateObject();
            cJSON_AddNumberToObject(entry, "start", r.start); cJSON_AddNumberToObject(entry, "count", r.count);
            cJSON_AddNumberToObject(entry, "offset", r.offset); cJSON_AddItemToArray(ranges, entry);
        }
        return sendJson(req, j);
    }
    if (route == "/diag/map") {
        const unsigned spokeCount = displaySpokes();
        double arm = 1, pixel = 0, spoke = runtime.spoke;
        if ((p.count("arm") && !number(p, "arm", 1, config.arms, arm)) ||
            (p.count("pix") && !number(p, "pix", 0, config.pixels - 1, pixel)) ||
            (p.count("spoke") && !number(p, "spoke", 0, spokeCount - 1, spoke))) return error(req, "Invalid mapping index");
        const uint64_t extent = sequence.logicalChannels(), oneSpoke = config.arms * config.pixels * 3;
        uint64_t channel = config.starts[unsigned(arm) - 1] - 1 + unsigned(pixel) * 3;
        const auto derived = fileSettings();
        if (config.autoFseq && derived.spokes)
            channel = fseqImageChannel(config.starts[unsigned(arm) - 1], unsigned(spoke), unsigned(pixel), derived.channelsPerSpoke);
        else if (extent != oneSpoke) channel += unsigned(spoke) * (extent % spokeCount == 0 ? extent / spokeCount : oneSpoke);
        cJSON* j = cJSON_CreateObject(); cJSON_AddNumberToObject(j, "absoluteChannel", channel + 1);
        cJSON* rgb = cJSON_AddArrayToObject(j, "rgb");
        for (unsigned c = 0; c < 3; ++c) cJSON_AddItemToArray(rgb, cJSON_CreateNumber(sequence.channel(channel + c)));
        return sendJson(req, j);
    }
    if (route == "/start" || route == "/play") {
        std::string why;
        if (!startPlayback(value(p, "path"), why)) return error(req, why);
        return success(req);
    }
    return error(req, "Not found", "404 Not Found");
}

esp_err_t post(httpd_req_t* req, const std::string& route, const Params& p) {
    if (route == "/wifi/scan") {
        std::string reason;
        if (!startWifiScan(reason)) return error(req, reason, "409 Conflict");
        return sendJson(req, wifiScanJson(), "202 Accepted");
    }
    Lock lock;
    double n;
    bool flag;
    if (route == "/sd/cancel") { cancelSdSpeedTest(); return sendJson(req, sdToolJson()); }
    if (sdToolRunning() && route != "/stop") return error(req, "Wait for the SD operation to finish", "409 Conflict");
    if (route == "/diag/flash-proof") {
        if (!boolean(p, "enable", flag)) return error(req, "Flash test enable must be 0 or 1");
        unsigned colorFrames = 0;
        if (p.count("colorFrames")) {
            if (!number(p, "colorFrames", 1, MaximumPulseColorFrames, n)) return error(req, "Choose a flash width from 1 to 3");
            colorFrames = static_cast<unsigned>(n);
        }
        std::string reason;
        if (flag && !startFlashProof(reason, colorFrames)) return error(req, reason, "409 Conflict");
        if (!flag) stopFlashProof();
        return sendJson(req, flashProofJson());
    }
    if (route == "/sd/scope") {
        double pin, interval;
        if (!number(p, "pin", SdScopeAllPins, 48, pin) || !number(p, "interval", 100, 10000, interval))
            return error(req, "Select an SD signal and sample interval");
        SdScopeOptions options; options.pin = int(pin); options.intervalUs = unsigned(interval);
        if (p.count("wifiOff") && !boolean(p, "wifiOff", options.wifiOff))
            return error(req, "wifiOff must be 0 or 1");
        std::string reason;
        if (!startSdScope(options, reason)) return error(req, reason, "409 Conflict");
        return sendJson(req, sdScopeJson(), "202 Accepted");
    }
    if (route == "/sd/read-test") {
        SdReadTestOptions options;
        options.path = value(p, "path");
        double mib = 4, block = 16384, phase = 0;
        bool continuous = false;
        if (!number(p, "mib", 1, 16, mib) ||
            (p.count("block") && !number(p, "block", 512, 16384, block)) ||
            (p.count("phase") && !number(p, "phase", 0, 3, phase)) ||
            (p.count("continuous") && !boolean(p, "continuous", continuous)))
            return error(req, "Invalid SD read test options");
        options.blockBytes = unsigned(block); options.delayPhase = unsigned(phase);
        options.continuousClock = continuous;
        if (p.count("keepBlink") && !boolean(p, "keepBlink", options.keepWhiteBlink))
            return error(req, "keepBlink must be 0 or 1");
        std::string reason;
        if (!startSdReadTest(unsigned(mib), options, reason)) return error(req, reason, "409 Conflict");
        return sendJson(req, sdToolJson(), "202 Accepted");
    }
    if (route == "/sd/format" || route == "/sd/speed") {
        const bool format = route == "/sd/format";
        if (format && value(p, "confirm") != "ERASE SD CARD")
            return error(req, "Explicit SD erase confirmation is required");
        n = 4;
        if (!format && (!number(p, "mib", 1, 16, n) || (n != 1 && n != 4 && n != 16)))
            return error(req, "Choose a 1, 4, or 16 MiB test");
        SdTimingOptions timing;
        double phase = 0;
        if (!format && ((p.count("phase") && !number(p, "phase", 0, 3, phase)) ||
            (p.count("continuous") && !boolean(p, "continuous", timing.continuousClock))))
            return error(req, "Invalid SD timing options");
        timing.delayPhase = unsigned(phase);
        std::string reason;
        if (!startSdTool(format, unsigned(n), reason, timing)) return error(req, reason, "409 Conflict");
        return sendJson(req, sdToolJson(), "202 Accepted");
    }
    if (route == "/stop") { stopPlayback(); return success(req); }
    if (route == "/fseq/radial") {
        const auto path = value(p, "path");
        std::string full;
        if (path.empty() || path == "/" || !sdPath(path, full) || !boolean(p, "tipFirst", flag))
            return error(req, "Choose a sequence path and pixel order");
        const esp_err_t saved = saveSequenceTipFirst(path, flag);
        if (saved != ESP_OK) return error(req, "Cannot save pixel order: " + std::string(esp_err_to_name(saved)));
        if (!strcasecmp(path.c_str(), runtime.filePath.c_str())) {
            runtime.sequenceTipFirst = flag;
            // Force the current frame to be repainted with its new radial map.
            ++runtime.playbackSession;
            ++runtime.generation;
            notifyDisplay();
        }
        return success(req);
    }
    if (route == "/pause") {
        if (runtime.mode != Mode::Playback) return error(req, "No sequence is playing", "409 Conflict");
        runtime.paused = !runtime.paused;
        return success(req);
    }
    if (route == "/reboot") { stopPlayback(); auto result = success(req); restartSoon(); return result; }
    if (route == "/logs/clear") { clearLogs(); return success(req); }
    if (route == "/wifi/password") {
        const auto target = value(p, "target");
        if (target != "router" && target != "ap") return error(req, "Select router or ap password");
        cJSON* j = cJSON_CreateObject();
        // Passwords are returned only on an explicit reveal request, never in
        // status, scans, HTML or logs. sendJson disables response caching.
        cJSON_AddStringToObject(j, "password", target == "router" ? config.password.c_str() : AccessPointPassword);
        return sendJson(req, j);
    }
    if (route == "/wifi/retry") {
        if (config.ssid.empty()) return error(req, "Save a router network name first");
        reconnectNetwork(); return success(req);
    }
    if (route == "/diag/strip-speed") {
        if (value(p, "action") == "stop") {
            cancelStripSpeedTest(); return sendJson(req, stripSpeedTestJson());
        }
        double seconds;
        const bool inHz = p.count("hz") != 0;
        if (!number(p, inHz ? "hz" : "mhz", inHz ? 2000000 : 2, inHz ? 40000000 : 40, n))
            return error(req, "Choose a listed clock rate and duration");
        const uint32_t clockHz = static_cast<uint32_t>(n) * (inHz ? 1U : 1000000U);
        if (!validStripClock(clockHz) ||
            !number(p, "seconds", 15, 120, seconds)) return error(req, "Choose a listed clock rate and duration");
        std::string reason;
        if (!startStripSpeedTest(clockHz, unsigned(seconds), reason))
            return error(req, reason, "409 Conflict");
        return sendJson(req, stripSpeedTestJson());
    }
    if (route == "/diag/dma") {
        if (!number(p, "mhz", 4, 16, n) || (n != 4 && n != 8 && n != 16)) return error(req, "Choose 4, 8, or 16 MHz");
        std::string reason;
        if (!startDmaTest(static_cast<uint32_t>(n) * 1000000, reason)) return error(req, reason, "409 Conflict");
        return sendJson(req, dmaTestJson());
    }
    if (route == "/diag/reset") {
        if (dmaTestRunning() || stripSpeedTestRunning()) return error(req, "Stop the speed test before resetting statistics", "409 Conflict");
        output.stats = {}; runtime.missedSpokes = 0; return success(req);
    }
    if (route == "/lanediag") { startDiagnostic(Mode::Connectors); return success(req); }
    if (route == "/led/arm-order") {
        if (!output.ready()) return error(req, "LED output unavailable", "409 Conflict");
        if (config.arms < 3) return error(req, "Enable at least three arms to identify their order", "409 Conflict");
        if (!config.brightness) return error(req, "Set brightness above 0% before the arm-order test", "409 Conflict");
        if (rpm() > 0) return error(req, "Stop the rotor before the arm-order test", "409 Conflict");
        startDiagnostic(Mode::ArmOrder);
        runtime.error.clear();
        log("Arm-order test: connectors 1, 5, 9, 13 in order for 60 seconds");
        return success(req);
    }
    if (route == "/led/spokes") {
        const auto pattern = value(p, "pattern");
        if (pattern != "quarters" && pattern != "alternating" && pattern != "alignment")
            return error(req, "Choose quarters, alternating or alignment");
        std::string reason;
        if (!startSpokeTest(pattern == "alignment" ? Mode::Alignment :
            pattern == "quarters" ? Mode::QuarterColors : Mode::AlternatingSpokes, reason))
            return error(req, reason, "409 Conflict");
        return success(req);
    }
    if (route == "/led/blink") {
        if (!output.ready()) return error(req, "LED output unavailable", "409 Conflict");
        if (!config.brightness) return error(req, "Set brightness above 0% before white blinking", "409 Conflict");
        startDiagnostic(Mode::WhiteBlink); runtime.error.clear();
        log("White blinking: 500 ms on / 500 ms off, saved brightness and center fade");
        return success(req);
    }
    if (route == "/led/solid") {
        const auto color = value(p, "color");
        std::array<uint8_t, 3> rgb{};
        if (color == "red") rgb = {255, 0, 0};
        else if (color == "green") rgb = {0, 255, 0};
        else if (color == "blue") rgb = {0, 0, 255};
        else if (color == "white") rgb = {255, 255, 255};
        else return error(req, "Choose red, green, blue, or white");
        if (!output.ready()) return error(req, "LED output unavailable", "409 Conflict");
        startDiagnostic(Mode::Solid); runtime.testColor = rgb; runtime.error.clear();
        return success(req);
    }
    if (route == "/led/signal") {
        double pin, pattern;
        if (!number(p, "pin", 0, 48, pin) || !number(p, "pattern", 0, 2, pattern))
            return error(req, "Choose a mapped LED signal and LOW, HIGH, or 1 Hz");
        bool allowed = int(pin) == BoardPins::Clock;
        for (unsigned arm = 0; arm < config.arms; ++arm) allowed |= int(pin) == BoardPins::ArmData[arm];
        if (!allowed) return error(req, "Only the LED clock and active arm data pins can be checked");
        if (!output.ready()) return error(req, "LED output unavailable", "409 Conflict");
        startDiagnostic(Mode::SignalCheck);
        runtime.signalPin = int(pin); runtime.signalPattern = uint8_t(pattern); runtime.signalHigh = false;
        runtime.error.clear();
        log("LED signal check: GPIO%d, pattern %u", runtime.signalPin, unsigned(runtime.signalPattern));
        return success(req);
    }
    if (route == "/colorfade") {
        if (!output.ready()) return error(req, "LED output unavailable; reboot after a DMA timeout", "409 Conflict");
        startDiagnostic(Mode::ColorFade);
        runtime.error.clear();
        log("All-arm color fade: %u arms, %u pixels per arm, %u%% brightness",
            unsigned(config.arms), unsigned(config.pixels), unsigned(config.brightness));
        return success(req);
    }
    if (route == "/halldiag" || route == "/armtest") {
        if (!boolean(p, "enable", flag)) return error(req, "enable must be 0 or 1");
        startDiagnostic(flag ? (route == "/halldiag" ? Mode::Hall : Mode::Arms) : Mode::Stopped);
        return success(req);
    }
    if (route == "/rpm") {
        if ((p.count("ppr") && value(p, "ppr") != "1") || (p.count("edge") && value(p, "edge") != "falling"))
            return error(req, "This build uses one falling-edge magnetic pulse per revolution");
        return success(req);
    }
    if (route == "/outmode") return error(req, "This board uses four independent outputs on a shared clock");
    if (route == "/sd/reinit") {
        std::string reason;
        if (!startSdMount(reason)) return error(req, reason, "409 Conflict");
        return sendJson(req, sdToolJson(), "202 Accepted");
    }
    if (route == "/sd/recover") {
        if (!requestSdRecovery("Manual lower-speed recovery", true))
            return error(req, "No lower setting available, or SD fallback is disabled/card unavailable", "409 Conflict");
        return sendJson(req, sdRecoveryJson(), "202 Accepted");
    }
    if (route == "/rm" || route == "/mkdir" || route == "/ren") {
        std::string full, destination;
        const auto path = value(p, "path");
        if (!sd.ready || path == "/" || !sdPath(path, full)) return error(req, "Invalid file path");
        stopPlayback(); sequence.close();
        if (!sequence.quiescent()) return error(req, "SD read is still finishing; retry shortly", "503 Service Unavailable");
        int result;
        if (route == "/mkdir") result = mkdir(full.c_str(), 0775);
        else if (route == "/ren") {
            if (!sdPath(value(p, "to"), destination) || destination == "/sdcard/") return error(req, "Invalid destination");
            struct stat info;
            if (stat(destination.c_str(), &info) == 0) return error(req, "Destination already exists");
            result = rename(full.c_str(), destination.c_str());
        } else {
            struct stat info;
            if (stat(full.c_str(), &info)) return error(req, "File not found", "404 Not Found");
            result = S_ISDIR(info.st_mode) ? rmdir(full.c_str()) : remove(full.c_str());
        }
        if (result != 0) {
            const int code = errno;
            if (sdIoError(code)) requestSdRecovery("SD file operation failed");
            return error(req, strerror(code));
        }
        return success(req);
    }
    Settings next = config;
    bool rebuild = false, network = false, remount = false;
    if (route == "/b") {
        if (!number(p, "value", 0, 100, n)) return error(req, "Brightness must be 0–100");
        next.brightness = n;
        if (p.count("center")) {
            if (!number(p, "center", 0, 100, n)) return error(req, "Center brightness must be 0-100");
            next.centerBrightness = n;
        }
        if (p.count("fade")) {
            if (!boolean(p, "fade", flag)) return error(req, "Center fade must be 0 or 1");
            next.centerDimming = flag;
        }
    } else if (route == "/duty") {
        if (config.autoDuty) return error(req, "Turn off Auto-calc before saving manual duty", "409 Conflict");
        if (!number(p, "percent", 0, 100, n)) return error(req, "Duty must be 0–100");
        next.duty = n;
    } else if (route == "/autoduty") {
        if (!boolean(p, "enable", flag)) return error(req, "Auto-calc enable must be 0 or 1");
        next.autoDuty = flag;
    } else if (route == "/fseq/settings") {
        if (!boolean(p, "enable", flag)) return error(req, "Use FSEQ settings must be 0 or 1");
        next.autoFseq = flag;
    } else if (route == "/speed") {
        if (!number(p, "fps", 1, 120, n)) return error(req, "FPS must be 1–120");
        next.fps = n;
    } else if (route == "/loop") {
        if (!boolean(p, "enable", flag)) return error(req, "Loop enable must be 0 or 1");
        next.loop = flag;
    } else if (route == "/mapcfg") {
        if (p.count("start")) { if (!number(p, "start", 1, 0x1000000, n)) return error(req, "Invalid start channel"); next.startChannel = n; }
        if (p.count("spokes")) { if (!number(p, "spokes", 1, 65535, n)) return error(req, "Invalid spoke count"); next.spokes = n; }
        if (p.count("arms")) { if (!number(p, "arms", 1, MaxArms, n)) return error(req, "Arms must be 1–4"); next.arms = n; }
        if (p.count("pixels")) { if (!number(p, "pixels", 1, MaxPixels, n)) return error(req, "Pixels must be 1–1024"); next.pixels = n; }
        if (p.count("usepa")) { if (!boolean(p, "usepa", flag)) return error(req, "Invalid per-arm option"); next.perArm = flag; }
        for (unsigned a = 0; a < MaxArms; ++a) {
            std::string key = "start" + std::to_string(a + 1);
            if (p.count(key)) { if (!number(p, key.c_str(), 1, 0x1000000, n)) return error(req, "Invalid per-arm start"); next.starts[a] = n; }
        }
        rebuild = next.pixels != config.pixels || next.arms != config.arms;
    } else if (route == "/autoplay" || route == "/watchdog" || route == "/bgeffect") {
        if (p.count("enable")) {
            if (!boolean(p, "enable", flag)) return error(req, "Invalid enable value");
            if (route == "/autoplay") next.autoplay = flag;
            else if (route == "/watchdog") next.watchdog = flag;
            else next.background = flag;
        }
        if (route == "/bgeffect" && p.count("path")) {
            std::string full; const auto path = value(p, "path");
            if (!path.empty() && (path.rfind("/BGEffects/", 0) != 0 || !sdPath(path, full))) return error(req, "Background must be in /BGEffects/");
            next.backgroundPath = path;
        }
    } else if (route == "/strobe") {
        if (p.count("enable")) { if (!boolean(p, "enable", flag)) return error(req, "Invalid strobe flag"); next.strobe = flag; }
        if (p.count("deg")) { if (!number(p, "deg", 0.1, 10, n, false)) return error(req, "Strobe width must be 0.1–10 degrees"); next.strobeWidth = n; }
        if (p.count("phase")) { if (!number(p, "phase", -360, 360, n, false)) return error(req, "Invalid phase"); next.phase = n; }
    } else if (route == "/armphase") {
        double arm;
        if (!number(p, "arm", 1, MaxArms, arm) || !number(p, "deg", -360, 360, n, false)) return error(req, "Invalid arm or phase");
        next.armPhase[unsigned(arm) - 1] = n;
    } else if (route == "/wifi") {
        if (value(p, "forget") == "1") { next.ssid.clear(); next.password.clear(); }
        else {
            if (p.count("ssid")) {
                next.ssid = value(p, "ssid");
                if (next.ssid != config.ssid) next.password.clear();
            }
            if (p.count("pass")) next.password = value(p, "pass");
            if (p.count("station")) next.hostname = value(p, "station");
        }
        if (next.ssid.size() > 32 || next.password.size() > 63 || next.hostname.size() > 63 ||
            next.hostname.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-") != std::string::npos)
            return error(req, "Invalid SSID, password length, or hostname");
        network = true;
    } else if (route == "/led/config") {
        if (!number(p, "protocol", 0, 1, n)) return error(req, "Choose SK9822 or APA102");
        next.ledProtocol = uint8_t(n);
        if (!number(p, "arms", 1, MaxArms, n)) return error(req, "Active arms must be 1-4");
        next.arms = uint8_t(n);
        if (!number(p, "pixels", 1, MaxPixels, n)) return error(req, "Pixels per arm must be 1-1024");
        next.pixels = uint16_t(n);
        rebuild = next.ledProtocol != config.ledProtocol || next.arms != config.arms || next.pixels != config.pixels;
    } else if (route == "/led/wiring") {
        const auto order = value(p, "order"), rotation = value(p, "rotation");
        if ((order != "cw" && order != "ccw") || (rotation != "cw" && rotation != "ccw"))
            return error(req, "Choose clockwise or counterclockwise for the lights and rotor, from the same viewing side");
        next.armClockwise = order == "cw";
        next.rotationClockwise = rotation == "cw";
        stopPlayback();
    } else if (route == "/sd/config") {
        if (!number(p, "mode", 0, 4, n) || (n != 0 && n != 1 && n != 4)) return error(req, "SD mode must be 0, 1, or 4");
        next.sdMode = n;
        if (!number(p, "freq", 400, 40000, n)) return error(req, "Invalid SD frequency");
        if (std::find(SdFrequencies.begin(), SdFrequencies.end(), n) == SdFrequencies.end())
            return error(req, "Choose 400 kHz, 20 MHz, or 40 MHz");
        next.sdFrequency = n; remount = true;
        if (p.count("fallback")) {
            if (!boolean(p, "fallback", flag)) return error(req, "fallback must be 0 or 1");
            next.sdFallback = flag;
        }
        if (p.count("recover")) {
            if (!boolean(p, "recover", flag)) return error(req, "recover must be 0 or 1");
            next.sdRecoverErrors = flag;
        }
    } else return error(req, "Not found", "404 Not Found");
    const Settings previous = config;
    config = next; normalizeSettings();
    if (rebuild && !reconfigureOutput()) { config = previous; return error(req, runtime.error, "500 Internal Server Error"); }
    esp_err_t err = saveSettings();
    if (err != ESP_OK) {
        config = previous;
        if (rebuild) reconfigureOutput();
        return error(req, std::string("Settings write failed: ") + esp_err_to_name(err), "500 Internal Server Error");
    }
    if (remount) {
        std::string reason;
        if (!startSdMount(reason)) return error(req, "Settings saved, but " + reason, "409 Conflict");
        return sendJson(req, sdToolJson(), "202 Accepted");
    }
    saveSettingsBackup();
    if (network) reconnectNetwork();
    ++runtime.generation; runtime.lastActivity = esp_timer_get_time(); notifyDisplay();
    return success(req);
}

esp_err_t route(httpd_req_t* req) {
    const std::string uri(req->uri);
    const size_t question = uri.find('?');
    const std::string path = uri.substr(0, question);
    Params params;
    if (question != std::string::npos && !parseParams(uri.substr(question + 1), params)) return error(req, "Malformed URL parameters");
    if (req->method == HTTP_GET) return get(req, path, params);
    if (path == "/upload" || path == "/fw/upload" || path == "/ota") return upload(req, path, params);
    if (req->content_len) {
        if (req->content_len > 4096) return error(req, "Settings request too large");
        std::string body(req->content_len, '\0');
        size_t received = 0;
        unsigned timeouts = 0;
        while (received < body.size()) {
            const int count = httpd_req_recv(req, body.data() + received, body.size() - received);
            if (count == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 3) continue;
            if (count <= 0) return ESP_FAIL;
            received += count;
        }
        if (!parseParams(body, params)) return error(req, "Malformed settings body");
    }
    return post(req, path, params);
}
}

cJSON* statusJson() {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "firmware", "native-esp-idf");
    cJSON_AddNumberToObject(j, "ledProtocol", config.ledProtocol);
    const auto* app = esp_app_get_description();
    cJSON_AddStringToObject(j, "firmwareVersion", app->version);
    cJSON_AddNumberToObject(j, "firmwareBuildNumber", LPOV_BUILD_NUMBER);
    cJSON_AddStringToObject(j, "firmwareBuild", LPOV_BUILD_TIME);
    cJSON_AddStringToObject(j, "firmwareBuildUtc", LPOV_BUILD_TIME_UTC);
    cJSON_AddItemToObject(j, "dmaTest", dmaTestJson());
    cJSON_AddItemToObject(j, "sdTools", sdToolJson());
    cJSON_AddBoolToObject(j, "playing", runtime.mode == Mode::Playback);
    cJSON_AddBoolToObject(j, "playbackComplete", runtime.playbackComplete);
    cJSON_AddBoolToObject(j, "paused", runtime.paused);
    cJSON* radial = cJSON_AddObjectToObject(j, "radialMapping");
    cJSON_AddStringToObject(radial, "path", runtime.filePath.c_str());
    cJSON_AddBoolToObject(radial, "tipFirst", runtime.sequenceTipFirst);
    cJSON_AddNumberToObject(j, "mode", static_cast<int>(runtime.mode));
    auto* blink = cJSON_AddObjectToObject(j, "whiteBlink");
    cJSON_AddBoolToObject(blink, "running", runtime.mode == Mode::WhiteBlink);
    cJSON_AddBoolToObject(blink, "on", runtime.whiteBlinkOn);
    cJSON_AddNumberToObject(blink, "transitions", runtime.whiteBlinkTransitions);
    cJSON_AddNumberToObject(blink, "period_ms", 1000);
    cJSON_AddStringToObject(j, "path", sequence.path.c_str());
    cJSON_AddStringToObject(j, "error", runtime.error.c_str());
    cJSON_AddItemToObject(j, "dutyControl", dutyControlJson());
    cJSON_AddItemToObject(j, "fileSettings", fileSettingsJson());
    cJSON_AddItemToObject(j, "flashProof", flashProofJson());
    cJSON_AddStringToObject(j, "playbackWarning", runtime.timingLimited ?
        (flashProofRunning() ? "Flash test cannot fit a complete color and black transfer at this RPM; pulses are skipped." :
        runtime.mode == Mode::Alignment ? "Alignment timing is too short for LED transfers. Reduce RPM." :
        config.autoDuty ? "Auto-calc cannot fit an LED update at this RPM. Reduce RPM or use fewer image spokes." :
        "Spoke timing is too short for LED transfers. Adjust duty, reduce RPM, or use fewer image spokes.") : "");
    cJSON_AddBoolToObject(j, "playbackDma", runtime.mode == Mode::Playback && output.dmaEnabled());
    cJSON_AddNumberToObject(j, "playbackClockHz", PlaybackDmaClockHz);
    cJSON_AddNumberToObject(j, "sdStalls", runtime.sdStalls);
    cJSON_AddNumberToObject(j, "frame", runtime.frame);
    cJSON_AddNumberToObject(j, "rpm", rpm());
    cJSON_AddNumberToObject(j, "pulseCount", hallSnapshot().count);
    cJSON_AddItemToObject(j, "hallFilter", hallFilterJson());
    cJSON_AddNumberToObject(j, "rpmPpr", 1);
    cJSON_AddStringToObject(j, "rpmEdge", "falling");
    cJSON_AddBoolToObject(j, "hallActive", !gpio_get_level(static_cast<gpio_num_t>(BoardPins::Hall)));
    cJSON_AddNumberToObject(j, "psramBytes", esp_psram_get_size());
    cJSON_AddNumberToObject(j, "freePsramBytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(j, "freeInternalBytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON* settings = cJSON_AddObjectToObject(j, "settings");
    cJSON_AddNumberToObject(settings, "brightness", config.brightness);
    cJSON_AddNumberToObject(settings, "centerBrightness", config.centerBrightness);
    cJSON_AddBoolToObject(settings, "centerDimming", config.centerDimming);
    cJSON_AddNumberToObject(settings, "duty", config.duty);
    cJSON_AddBoolToObject(settings, "autoDuty", config.autoDuty);
    cJSON_AddBoolToObject(settings, "autoFseq", config.autoFseq);
    cJSON_AddNumberToObject(settings, "fps", config.fps);
    cJSON_AddNumberToObject(settings, "start", config.startChannel);
    cJSON_AddNumberToObject(settings, "spokes", config.spokes);
    cJSON_AddNumberToObject(settings, "arms", config.arms);
    cJSON_AddBoolToObject(settings, "armClockwise", config.armClockwise);
    cJSON_AddBoolToObject(settings, "rotationClockwise", config.rotationClockwise);
    cJSON_AddNumberToObject(settings, "pixels", config.pixels);
    cJSON_AddBoolToObject(settings, "usepa", config.perArm);
    cJSON_AddBoolToObject(settings, "autoplay", config.autoplay);
    cJSON_AddBoolToObject(settings, "loop", config.loop);
    cJSON_AddBoolToObject(settings, "watchdog", config.watchdog);
    cJSON_AddBoolToObject(settings, "background", config.background);
    cJSON_AddStringToObject(settings, "backgroundPath", config.backgroundPath.c_str());
    cJSON_AddBoolToObject(settings, "strobe", config.strobe);
    cJSON_AddNumberToObject(settings, "strobeWidth", config.strobeWidth);
    cJSON_AddNumberToObject(settings, "phase", config.phase);
    cJSON* starts = cJSON_AddArrayToObject(settings, "starts");
    cJSON* phases = cJSON_AddArrayToObject(settings, "armPhase");
    for (unsigned a = 0; a < MaxArms; ++a) {
        cJSON_AddItemToArray(starts, cJSON_CreateNumber(config.starts[a]));
        cJSON_AddItemToArray(phases, cJSON_CreateNumber(config.armPhase[a]));
    }
    cJSON* wifi = cJSON_AddObjectToObject(j, "wifi");
    cJSON_AddStringToObject(wifi, "ssid", config.ssid.c_str());
    cJSON_AddBoolToObject(wifi, "passwordSaved", !config.password.empty());
    cJSON_AddStringToObject(wifi, "apSsid", AccessPointName);
    cJSON_AddStringToObject(wifi, "station", config.hostname.c_str());
    cJSON_AddStringToObject(wifi, "ip", stationIp().c_str());
    addNetworkStatus(wifi);
    cJSON* storage = cJSON_AddObjectToObject(j, "sd");
    cJSON_AddBoolToObject(storage, "ready", sd.ready);
    cJSON_AddNumberToObject(storage, "currentWidth", sd.width);
    cJSON_AddNumberToObject(storage, "freq", sd.frequency);
    cJSON_AddNumberToObject(storage, "baseFreq", config.sdFrequency);
    cJSON_AddNumberToObject(storage, "desiredMode", config.sdMode);
    cJSON_AddBoolToObject(storage, "fallback", config.sdFallback);
    cJSON_AddBoolToObject(storage, "recoverErrors", config.sdRecoverErrors);
    cJSON_AddItemToObject(storage, "recovery", sdRecoveryJson());
    cJSON_AddItemToObject(storage, "ioErrors", sdIoDiagnosticsJson());
    cJSON_AddNumberToObject(storage, "bytes", sd.bytes);
    return j;
}
void startWeb() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 12288; cfg.core_id = 0;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10; cfg.send_wait_timeout = 10;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));
    httpd_uri_t handler = {};
    handler.uri = "/*"; handler.handler = route; handler.method = HTTP_GET;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &handler));
    handler.method = HTTP_POST;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &handler));
}
}
