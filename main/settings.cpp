#include "app.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "nvs.h"
#include "esp_mac.h"

namespace pov {
Settings config;
namespace {
nvs_handle_t handle;
void upgradeLoadedHostname() {
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK)
        config.hostname = hostnameAfterUpgrade(config.hostname, mac);
}
enum class Kind { U8, U16, U32, Bool, Float, Text };
struct Field { const char* key; Kind kind; void* value; const char* backup; };
std::vector<Field> fields() {
    return {
        {"brightness", Kind::U8, &config.brightness, "brightness"},
        {"centerbright", Kind::U8, &config.centerBrightness, "centerbright"},
        {"centerdim", Kind::Bool, &config.centerDimming, "centerdim"},
        {"duty", Kind::U8, &config.duty, "duty"},
        {"autoduty", Kind::Bool, &config.autoDuty, "autoduty"},
        {"autofseq", Kind::Bool, &config.autoFseq, "autofseq"},
        {"arms", Kind::U8, &config.arms, "arms"},
        {"arm_cw", Kind::Bool, &config.armClockwise, "arm_cw"},
        {"rotation_cw", Kind::Bool, &config.rotationClockwise, "rotation_cw"},
        {"fps", Kind::U16, &config.fps, "fps"},
        {"spokes", Kind::U16, &config.spokes, "spokes"},
        {"pixels", Kind::U16, &config.pixels, "pixels"},
        {"ledproto", Kind::U8, &config.ledProtocol, "ledproto"},
        {"startch", Kind::U32, &config.startChannel, "startch"},
        {"sdmode", Kind::U8, &config.sdMode, "sdmode"},
        {"sdfreq", Kind::U32, &config.sdFrequency, "sdfreq"},
        {"sdfallback", Kind::Bool, &config.sdFallback, "sdfallback"},
        {"sdrecover", Kind::Bool, &config.sdRecoverErrors, "sdrecover"},
        {"autoplay", Kind::Bool, &config.autoplay, "autoplay"},
        {"loop", Kind::Bool, &config.loop, "loop"},
        {"watchdog", Kind::Bool, &config.watchdog, "watchdog"},
        {"bge_enable", Kind::Bool, &config.background, "bge_enable"},
        {"usepa", Kind::Bool, &config.perArm, "usepa"},
        {"strb_e", Kind::Bool, &config.strobe, "strb_e"},
        {"strb_deg", Kind::Float, &config.strobeWidth, "strb_deg"},
        {"strb_ph", Kind::Float, &config.phase, "strb_ph"},
        {"start1", Kind::U32, &config.starts[0], "start1"},
        {"start2", Kind::U32, &config.starts[1], "start2"},
        {"start3", Kind::U32, &config.starts[2], "start3"},
        {"start4", Kind::U32, &config.starts[3], "start4"},
        {"phase1", Kind::Float, &config.armPhase[0], "phase1"},
        {"phase2", Kind::Float, &config.armPhase[1], "phase2"},
        {"phase3", Kind::Float, &config.armPhase[2], "phase3"},
        {"phase4", Kind::Float, &config.armPhase[3], "phase4"},
        {"sta_ssid", Kind::Text, &config.ssid, "ssid"},
        {"sta_pass", Kind::Text, &config.password, "pass"},
        {"station", Kind::Text, &config.hostname, "station"},
        {"bge_path", Kind::Text, &config.backgroundPath, "bge_path"},
    };
}
esp_err_t readField(const Field& f) {
    uint8_t flag;
    size_t size;
    switch (f.kind) {
    case Kind::U8: return nvs_get_u8(handle, f.key, static_cast<uint8_t*>(f.value));
    case Kind::U16: return nvs_get_u16(handle, f.key, static_cast<uint16_t*>(f.value));
    case Kind::U32: return nvs_get_u32(handle, f.key, static_cast<uint32_t*>(f.value));
    case Kind::Bool: {
        esp_err_t err = nvs_get_u8(handle, f.key, &flag);
        if (err == ESP_OK) *static_cast<bool*>(f.value) = flag != 0;
        return err;
    }
    case Kind::Float:
        size = sizeof(float);
        return nvs_get_blob(handle, f.key, f.value, &size);
    case Kind::Text: {
        size = 0;
        esp_err_t err = nvs_get_str(handle, f.key, nullptr, &size);
        if (err != ESP_OK || size > 512) return err == ESP_OK ? ESP_ERR_INVALID_SIZE : err;
        std::vector<char> text(size);
        err = nvs_get_str(handle, f.key, text.data(), &size);
        if (err == ESP_OK) *static_cast<std::string*>(f.value) = text.data();
        return err;
    }
    }
    return ESP_FAIL;
}
esp_err_t writeField(const Field& f) {
    switch (f.kind) {
    case Kind::U8: return nvs_set_u8(handle, f.key, *static_cast<uint8_t*>(f.value));
    case Kind::U16: return nvs_set_u16(handle, f.key, *static_cast<uint16_t*>(f.value));
    case Kind::U32: return nvs_set_u32(handle, f.key, *static_cast<uint32_t*>(f.value));
    case Kind::Bool: return nvs_set_u8(handle, f.key, *static_cast<bool*>(f.value));
    case Kind::Float: return nvs_set_blob(handle, f.key, f.value, sizeof(float));
    case Kind::Text: return nvs_set_str(handle, f.key, static_cast<std::string*>(f.value)->c_str());
    }
    return ESP_FAIL;
}
esp_err_t readRadialFiles(cJSON*& files) {
    files = nullptr;
    size_t size = 0;
    esp_err_t err = nvs_get_str(handle, "radial_files", nullptr, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        files = cJSON_CreateObject();
        return files ? ESP_OK : ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) return err;
    if (!size || size > 8192) return ESP_ERR_INVALID_SIZE;
    std::vector<char> text(size);
    if ((err = nvs_get_str(handle, "radial_files", text.data(), &size)) != ESP_OK) return err;
    files = cJSON_Parse(text.data());
    if (!cJSON_IsObject(files)) { cJSON_Delete(files); files = nullptr; return ESP_ERR_INVALID_STATE; }
    return ESP_OK;
}
}

bool loadSequenceTipFirst(const std::string& path) {
    cJSON* files;
    const esp_err_t err = readRadialFiles(files);
    if (err != ESP_OK) { log("Cannot load sequence pixel order: %s", esp_err_to_name(err)); return false; }
    // FAT paths are case-insensitive, as is this cJSON lookup.
    const bool tipFirst = cJSON_IsTrue(cJSON_GetObjectItem(files, path.c_str()));
    cJSON_Delete(files);
    return tipFirst;
}
esp_err_t saveSequenceTipFirst(const std::string& path, bool tipFirst) {
    cJSON* files;
    esp_err_t err = readRadialFiles(files);
    if (err != ESP_OK) return err;
    cJSON_DeleteItemFromObject(files, path.c_str());
    if (!cJSON_AddBoolToObject(files, path.c_str(), tipFirst)) { cJSON_Delete(files); return ESP_ERR_NO_MEM; }
    char* text = cJSON_PrintUnformatted(files);
    cJSON_Delete(files);
    if (!text) return ESP_ERR_NO_MEM;
    err = strlen(text) < 8192 ? nvs_set_str(handle, "radial_files", text) : ESP_ERR_INVALID_SIZE;
    cJSON_free(text);
    if (err == ESP_OK) err = nvs_commit(handle);
    return err;
}

void normalizeSettings() {
    config.brightness = std::min<unsigned>(config.brightness, 100);
    config.centerBrightness = std::min<unsigned>(config.centerBrightness, 100);
    config.duty = std::min<unsigned>(config.duty, 100);
    config.arms = std::clamp<unsigned>(config.arms, 1, MaxArms);
    config.pixels = std::clamp<unsigned>(config.pixels, 1, MaxPixels);
    if (config.ledProtocol > 1) config.ledProtocol = 0;
    config.fps = std::clamp<unsigned>(config.fps, 1, 120);
    config.spokes = std::max<uint16_t>(config.spokes, 1);
    config.startChannel = std::clamp<uint32_t>(config.startChannel, 1, 0x1000000);
    if (config.sdMode != 1 && config.sdMode != 4) config.sdMode = 0;
    config.sdFrequency = normalizedSdFrequency(config.sdFrequency);
    if (!std::isfinite(config.strobeWidth)) config.strobeWidth = 3;
    config.strobeWidth = std::clamp(config.strobeWidth, 0.1f, 10.0f);
    if (!std::isfinite(config.phase)) config.phase = 0;
    config.phase = std::fmod(config.phase, 360.0f);
    for (unsigned arm = 0; arm < MaxArms; ++arm) {
        if (!config.perArm) config.starts[arm] = config.startChannel + arm * config.pixels * 3;
        else config.starts[arm] = std::clamp<uint32_t>(config.starts[arm], 1, 0x1000000);
        if (!std::isfinite(config.armPhase[arm])) config.armPhase[arm] = 0;
        config.armPhase[arm] = std::fmod(config.armPhase[arm], 360.0f);
    }
    if (config.hostname.empty()) config.hostname = DefaultHostname;
}

esp_err_t loadSettings() {
    esp_err_t err = nvs_open("display", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    for (const auto& f : fields()) readField(f);
    upgradeLoadedHostname();
    // The Arduino version stored arm 1 in startch even in per-arm mode.
    nvs_type_t startType;
    if (nvs_find_key(handle, "start1", &startType) != ESP_OK) config.starts[0] = config.startChannel;
    uint8_t revision = 0;
    nvs_get_u8(handle, "pcb_rev", &revision);
    if (revision != 1) {
        config.arms = BoardPins::InitialArms;
        config.perArm = false;
        // Commit topology before importing an older SD backup.
        if ((err = nvs_set_u8(handle, "arms", config.arms)) != ESP_OK) return err;
        if ((err = nvs_set_u8(handle, "usepa", 0)) != ESP_OK) return err;
    }
    if ((err = nvs_set_u8(handle, "pcb_rev", 1)) != ESP_OK) return err;
    if ((err = nvs_set_u8(handle, "ppr", 1)) != ESP_OK) return err;
    if ((err = nvs_set_u8(handle, "hedge", 0)) != ESP_OK) return err;
    if ((err = nvs_set_u8(handle, "outmode", 1)) != ESP_OK) return err;
    normalizeSettings();
    return nvs_commit(handle);
}

esp_err_t saveSettings() {
    normalizeSettings();
    for (const auto& f : fields()) {
        esp_err_t err = writeField(f);
        if (err != ESP_OK) return err;
    }
    return nvs_commit(handle);
}

void restoreSettingsBackup() {
    FILE* file = fopen("/sdcard/config/settings.ini", "rb");
    if (!file) return;
    char line[640];
    const auto entries = fields();
    while (fgets(line, sizeof(line), file)) {
        char* equals = strchr(line, '=');
        if (!equals) continue;
        *equals++ = '\0';
        equals[strcspn(equals, "\r\n")] = '\0';
        for (const auto& f : entries) {
            if (strcmp(line, f.backup) != 0) continue;
            nvs_type_t type;
            if (nvs_find_key(handle, f.key, &type) == ESP_OK) break;
            const unsigned long number = strtoul(equals, nullptr, 10);
            switch (f.kind) {
            case Kind::U8: *static_cast<uint8_t*>(f.value) = std::min<unsigned long>(number, 255); break;
            case Kind::U16: *static_cast<uint16_t*>(f.value) = std::min<unsigned long>(number, 65535); break;
            case Kind::U32: *static_cast<uint32_t*>(f.value) = number; break;
            case Kind::Bool: *static_cast<bool*>(f.value) = number != 0; break;
            case Kind::Float: *static_cast<float*>(f.value) = strtof(equals, nullptr); break;
            case Kind::Text: *static_cast<std::string*>(f.value) = equals; break;
            }
        }
    }
    fclose(file);
    upgradeLoadedHostname();
    normalizeSettings();
}

bool saveSettingsBackup() {
    if (runtime.maintenance) return false;
    // NVS is the authoritative saved copy. Do not wait on SD while a frame
    // worker or rotation test is active: the caller holds the display's state lock.
    sequence.quiescent();
    if (sequence.loading() || usesSpokeTiming(runtime.mode) || stripSpeedTestRunning()) return false;
    if (!sd.ready) return false;
    FILE* file = fopen("/sdcard/config/settings.tmp", "wb");
    if (!file) return false;
    for (const auto& f : fields()) {
        fprintf(file, "%s=", f.backup);
        switch (f.kind) {
        case Kind::U8: fprintf(file, "%u", *static_cast<uint8_t*>(f.value)); break;
        case Kind::U16: fprintf(file, "%u", *static_cast<uint16_t*>(f.value)); break;
        case Kind::U32: fprintf(file, "%lu", static_cast<unsigned long>(*static_cast<uint32_t*>(f.value))); break;
        case Kind::Bool: fprintf(file, "%d", *static_cast<bool*>(f.value)); break;
        case Kind::Float: fprintf(file, "%.3f", *static_cast<float*>(f.value)); break;
        case Kind::Text: fprintf(file, "%s", static_cast<std::string*>(f.value)->c_str()); break;
        }
        fputc('\n', file);
    }
    bool ok = ferror(file) == 0;
    if (fclose(file) != 0) ok = false;
    if (!ok) return false;
    remove("/sdcard/config/settings.ini");
    return rename("/sdcard/config/settings.tmp", "/sdcard/config/settings.ini") == 0;
}
}
