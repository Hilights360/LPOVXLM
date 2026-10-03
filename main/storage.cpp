#include "app.hpp"
#include "sd_recovery.hpp"
#include <algorithm>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "esp_ota_ops.h"
#include "sdmmc_cmd.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "ff.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "hal/sdmmc_ll.h"
#include "soc/sdmmc_struct.h"

namespace pov {
SdState sd;
static sdmmc_card_t* card;

namespace {
// Keep the original host error: FatFs maps CRC and timeout failures to EIO.
// Install the callback only after successful initialization, so expected probe
// responses are not counted as transfer failures. Preserve it across remounts.
portMUX_TYPE ioErrorMutex = portMUX_INITIALIZER_UNLOCKED;
struct SdIoError {
    uint32_t count = 0, command = 0;
    size_t bytes = 0;
    esp_err_t code = ESP_OK;
    unsigned width = 0, frequency = 0;
    int64_t at = 0;
} lastIoError;
std::array<SdIoError, 8> ioHistory{};
unsigned ioWidth = 0, ioFrequency = 0;
esp_err_t observedSdTransaction(int slot, sdmmc_command_t* cmd) {
    const esp_err_t result = sdmmc_host_do_transaction(slot, cmd);
    const esp_err_t error = result == ESP_OK ? cmd->error : result;
    if (error != ESP_OK) {
        const int64_t at = esp_timer_get_time();
        portENTER_CRITICAL(&ioErrorMutex);
        ++lastIoError.count;
        lastIoError.command = cmd->opcode; lastIoError.bytes = cmd->datalen;
        lastIoError.code = error; lastIoError.at = at;
        lastIoError.width = ioWidth; lastIoError.frequency = ioFrequency;
        ioHistory[(lastIoError.count - 1) % ioHistory.size()] = lastIoError;
        portEXIT_CRITICAL(&ioErrorMutex);
        log("SD: %s on CMD%u, %u bytes at %u-bit/%u kHz", esp_err_to_name(error),
            unsigned(cmd->opcode), unsigned(cmd->datalen), ioWidth, ioFrequency);
    }
    return result;
}
bool prepareSdBus() {
    // Release all six lines between attempts, including when changing from
    // 4-bit to 1-bit. The host only configures CMD/D0 pull-ups in 1-bit mode;
    // DAT1-3 must also stay pulled up, especially DAT3 during mode selection.
    // These internal pulls supplement the PCB's external 10k resistors.
    gpio_config_t pins = {};
    pins.mode = GPIO_MODE_INPUT;
    pins.pin_bit_mask = 1ULL << BoardPins::SdClk;
    esp_err_t err = gpio_config(&pins);
    if (err == ESP_OK) {
        pins.pin_bit_mask = (1ULL << BoardPins::SdCmd) | (1ULL << BoardPins::SdD0) |
            (1ULL << BoardPins::SdD1) | (1ULL << BoardPins::SdD2) | (1ULL << BoardPins::SdD3);
        pins.pull_up_en = GPIO_PULLUP_ENABLE;
        err = gpio_config(&pins);
    }
    if (err != ESP_OK) {
        log("SD: preparing GPIOs failed: %s", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    const int cmd = gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdCmd));
    const int d0 = gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdD0));
    const int d1 = gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdD1));
    const int d2 = gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdD2));
    const int d3 = gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdD3));
    log("SD: idle with pull-ups CMD=%d D0=%d D1=%d D2=%d D3=%d (expected all 1)",
        cmd, d0, d1, d2, d3);
    if (!(cmd && d0 && d1 && d2 && d3))
        log("SD: a CMD/DAT line reads LOW before initialization; check card power and connections");
    return true;
}
// sdmmc_host_set_card_clk() programs a 100 ms data-read timeout every time the
// clock is set, and IDF exposes no setting for it, so reapply the longer one
// after each successful mount. The register counts card clocks and saturates at
// 24 bits, which is 838 ms at 20 MHz, so every supported rate fits.
void applySdDataTimeout(unsigned frequencyKHz) {
    if (!frequencyKHz) return;
    sdmmc_ll_set_data_timeout(&SDMMC, SdDataTimeoutMs * frequencyKHz);
    log("SD: data-read timeout %u ms at %u kHz", SdDataTimeoutMs, frequencyKHz);
}
sdmmc_slot_config_t sdSlot(unsigned width) {
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = width;
    slot.cd = BoardPins::SdCd >= 0 ? static_cast<gpio_num_t>(BoardPins::SdCd) : SDMMC_SLOT_NO_CD;
    slot.clk = static_cast<gpio_num_t>(BoardPins::SdClk);
    slot.cmd = static_cast<gpio_num_t>(BoardPins::SdCmd);
    slot.d0 = static_cast<gpio_num_t>(BoardPins::SdD0);
    slot.d1 = width == 4 ? static_cast<gpio_num_t>(BoardPins::SdD1) : GPIO_NUM_NC;
    slot.d2 = width == 4 ? static_cast<gpio_num_t>(BoardPins::SdD2) : GPIO_NUM_NC;
    slot.d3 = width == 4 ? static_cast<gpio_num_t>(BoardPins::SdD3) : GPIO_NUM_NC;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    return slot;
}
}

cJSON* sdIoDiagnosticsJson() {
    portENTER_CRITICAL(&ioErrorMutex);
    const SdIoError snapshot = lastIoError;
    const auto history = ioHistory;
    portEXIT_CRITICAL(&ioErrorMutex);
    auto* j = cJSON_CreateObject();
    cJSON_AddNumberToObject(j, "count", snapshot.count);
    cJSON_AddStringToObject(j, "lastError", snapshot.count ? esp_err_to_name(snapshot.code) : "");
    cJSON_AddNumberToObject(j, "lastErrorCode", snapshot.code);
    cJSON_AddNumberToObject(j, "command", snapshot.command);
    cJSON_AddNumberToObject(j, "transferBytes", snapshot.bytes);
    cJSON_AddNumberToObject(j, "busWidth", snapshot.width);
    cJSON_AddNumberToObject(j, "clockKHz", snapshot.frequency);
    cJSON_AddNumberToObject(j, "uptime_ms", snapshot.at / 1000);
    auto* recent = cJSON_AddArrayToObject(j, "recent");
    const uint32_t count = std::min<uint32_t>(snapshot.count, history.size());
    for (uint32_t i = snapshot.count - count; i < snapshot.count; ++i) {
        const auto& entry = history[i % history.size()];
        auto* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "count", entry.count);
        cJSON_AddStringToObject(item, "error", esp_err_to_name(entry.code));
        cJSON_AddNumberToObject(item, "command", entry.command);
        cJSON_AddNumberToObject(item, "transferBytes", entry.bytes);
        cJSON_AddNumberToObject(item, "busWidth", entry.width);
        cJSON_AddNumberToObject(item, "clockKHz", entry.frequency);
        cJSON_AddNumberToObject(item, "uptime_ms", entry.at / 1000);
        cJSON_AddItemToArray(recent, item);
    }
    return j;
}

// The SD worker reserves exclusive access before changing these volatile settings.
esp_err_t configureSdReadTiming(unsigned phase, bool continuousClock) {
    if (!card || phase > 3) return ESP_ERR_INVALID_STATE;
    esp_err_t err = sdmmc_host_set_input_delay(card->host.slot, static_cast<sdmmc_delay_phase_t>(phase));
    if (err == ESP_OK) err = sdmmc_host_set_cclk_always_on(card->host.slot, continuousClock);
    return err;
}
esp_err_t restoreSdReadTiming() {
    if (!card) return ESP_ERR_INVALID_STATE;
    // SD memory mounts use gated clocks and the host's configured input phase.
    const esp_err_t phase = sdmmc_host_set_input_delay(card->host.slot, card->host.input_delay_phase);
    const esp_err_t clock = sdmmc_host_set_cclk_always_on(card->host.slot, false);
    return phase != ESP_OK ? phase : clock;
}

bool sdPath(const std::string& relative, std::string& result) {
    if (relative.empty() || relative[0] != '/' || relative.size() > 240) return false;
    if (relative.find_first_of("\\:\r\n") != std::string::npos || relative.find('\0') != std::string::npos) return false;
    size_t start = 1;
    while (start <= relative.size()) {
        size_t end = relative.find('/', start);
        if (end == std::string::npos) end = relative.size();
        const auto part = relative.substr(start, end - start);
        if (part == ".." || part == ".") return false;
        start = end + 1;
    }
    result = "/sdcard" + relative;
    return true;
}

bool unmountSd() {
    sequence.close();
    if (!sequence.quiescent()) {
        log("SD: cancelled read is still finishing; mount retry deferred");
        return false;
    }
    if (card && esp_vfs_fat_sdcard_unmount("/sdcard", card) != ESP_OK) return false;
    card = nullptr;
    sd = {};
    return true;
}

static bool mountSdHardware(unsigned desiredMode, unsigned maxFrequency, bool fallback,
                            SdState& mounted, const SdState* failed = nullptr) {
    log("SD: GPIO CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d CD=%d",
        BoardPins::SdClk, BoardPins::SdCmd, BoardPins::SdD0,
        BoardPins::SdD1, BoardPins::SdD2, BoardPins::SdD3, BoardPins::SdCd);
    if constexpr (BoardPins::SdCd >= 0) {
        gpio_set_direction(static_cast<gpio_num_t>(BoardPins::SdCd), GPIO_MODE_INPUT);
        gpio_set_pull_mode(static_cast<gpio_num_t>(BoardPins::SdCd), GPIO_PULLUP_ONLY);
        if (gpio_get_level(static_cast<gpio_num_t>(BoardPins::SdCd))) {
            log("SD: no card detected");
            return false;
        }
    } else {
        log("SD: no card-detect GPIO; probing card over SDMMC");
    }
    const auto profiles = sdProfiles(desiredMode, maxFrequency, fallback);
    const size_t begin = failed ? profiles.after({failed->width, failed->profileFrequency}) : 0;
    for (size_t i = begin; i < profiles.count; ++i) {
        const auto [width, rate] = profiles.values[i];
        if (failed) { Lock lock; sdRecoveryAttempt(width, rate); }
        if (!prepareSdBus()) return false;
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = rate;
        sdmmc_slot_config_t slot = sdSlot(width);
        esp_vfs_fat_sdmmc_mount_config_t mount = {};
        mount.format_if_mount_failed = false;
        mount.max_files = 6;
        mount.allocation_unit_size = 16 * 1024;
        esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &card);
        if (err == ESP_OK) {
            int actualKHz = static_cast<int>(rate);
            sdmmc_host_get_real_freq(host.slot, &actualKHz);
            mounted = {true, width, static_cast<unsigned>(actualKHz),
                  uint64_t(card->csd.capacity) * card->csd.sector_size, rate};
            ioWidth = width; ioFrequency = actualKHz;
            applySdDataTimeout(actualKHz);
            card->host.do_transaction = observedSdTransaction;
            mkdir("/sdcard/config", 0775);
            mkdir("/sdcard/BGEffects", 0775);
            log("SD: %u-bit at %d kHz, %llu MB", width, actualKHz,
                static_cast<unsigned long long>(mounted.bytes / (1024 * 1024)));
            return true;
        }
        card = nullptr;
        log("SD: %u-bit/%u kHz failed: %s", width, rate, esp_err_to_name(err));
    }
    return false;
}

bool mountSd() {
    if (!unmountSd()) return false;
    return mountSdHardware(config.sdMode, config.sdFrequency, config.sdFallback, sd);
}

bool recoverSdCard(unsigned mode, unsigned frequency, const SdState& failed,
                   SdState& mounted, std::string& error) {
    // Caller has reserved exclusive access and drained outstanding reads.
    if (card) {
        const esp_err_t err = esp_vfs_fat_sdcard_unmount("/sdcard", card);
        if (err != ESP_OK) { error = "SD recovery cannot unmount: " + std::string(esp_err_to_name(err)); return false; }
        card = nullptr;
    }
    if (mountSdHardware(mode, frequency, true, mounted, &failed)) return true;
    error = "No remaining SD setting worked, down to 1-bit/400 kHz. Check the card and connections, then Retry mount.";
    return false;
}

bool remountSdCard(unsigned mode, unsigned frequency, bool fallback, SdState& mounted, std::string& error) {
    if (card) {
        const esp_err_t err = esp_vfs_fat_sdcard_unmount("/sdcard", card);
        if (err != ESP_OK) { error = "Cannot unmount SD card: " + std::string(esp_err_to_name(err)); return false; }
        card = nullptr;
    }
    if (mountSdHardware(mode, frequency, fallback, mounted)) return true;
    error = "SD mount failed at all permitted settings; check the card and connections";
    return false;
}

bool detachSdForScope(std::string& error) {
    // The SD worker has blocked new I/O and drained the frame reader.
    if (card) {
        const auto err = esp_vfs_fat_sdcard_unmount("/sdcard", card);
        if (err != ESP_OK) { error = "Cannot detach SD host: " + std::string(esp_err_to_name(err)); return false; }
        card = nullptr;
    }
    return true;
}

// Exclusive access is reserved by the SD tool worker before entering here.
// Work outside stateMutex and publish the resulting mount only after finishing.
bool formatSdCard(unsigned mode, unsigned frequency, bool fallback, SdState& mounted, std::string& error) {
    if (card) {
        const esp_err_t err = esp_vfs_fat_sdcard_unmount("/sdcard", card);
        if (err != ESP_OK) { error = "Cannot unmount SD card: " + std::string(esp_err_to_name(err)); return false; }
        card = nullptr;
    }
    if (!prepareSdBus()) { error = "Cannot initialize SD pins"; return false; }
    // SD initialization probes at 400 kHz, then uses at most 20 MHz in 1-bit
    // mode. Avoid IDF's short output phase delay at custom 1-10 MHz clocks.
    // No write precedes successful card initialization.
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = std::min(frequency, SdDefaultFrequency);
    sdmmc_slot_config_t slot = sdSlot(1);
    sdmmc_card_t raw = {};
    esp_err_t err = sdmmc_host_init();
    if (err != ESP_OK) { error = esp_err_to_name(err); return false; }
    err = sdmmc_host_init_slot(host.slot, &slot);
    if (err == ESP_OK) err = sdmmc_card_init(&host, &raw);
    BYTE drive = FF_DRV_NOT_USED;
    void* work = nullptr;
    if (err == ESP_OK) {
        work = heap_caps_malloc(4096, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        err = work ? ff_diskio_get_drive(&drive) : ESP_ERR_NO_MEM;
    }
    if (err == ESP_OK) {
        ff_diskio_register_sdmmc(drive, &raw);
        // Match IDF's SD partitioning procedure: one partition spanning the
        // card, then a FAT filesystem. Nothing touches ESP flash or NVS.
        LBA_t sizes[] = {100, 0, 0, 0};
        FRESULT result = f_fdisk(drive, sizes, work);
        const char path[] = {static_cast<char>('0' + drive), ':', 0};
        const MKFS_PARM options = {FM_ANY, 2, 0, 0, 16 * 1024};
        if (result == FR_OK) result = f_mkfs(path, &options, work, 4096);
        ff_diskio_unregister(drive);
        if (result != FR_OK) { error = "SD format failed (FatFs " + std::to_string(result) + ")"; err = ESP_FAIL; }
    }
    free(work);
    sdmmc_host_deinit();
    if (err != ESP_OK) {
        if (error.empty()) error = "SD initialization/format failed: " + std::string(esp_err_to_name(err));
        return false;
    }
    if (!mountSdHardware(mode, frequency, fallback, mounted)) { error = "SD formatted, but remount failed; retry mounting"; return false; }
    return true;
}

esp_err_t installFirmware(FILE* file, size_t size) {
    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (!target || !size || size > target->size) return ESP_ERR_INVALID_SIZE;
    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(target, size, &handle);
    if (err != ESP_OK) return err;
    uint8_t buffer[4096];
    size_t remaining = size;
    while (remaining) {
        const size_t count = fread(buffer, 1, std::min(remaining, sizeof(buffer)), file);
        if (!count) { err = ESP_FAIL; break; }
        err = esp_ota_write(handle, buffer, count);
        if (err != ESP_OK) break;
        remaining -= count;
        vTaskDelay(1);
    }
    if (err != ESP_OK) { esp_ota_abort(handle); return err; }
    err = esp_ota_end(handle);
    if (err == ESP_OK) err = esp_ota_set_boot_partition(target);
    return err;
}

void installSdFirmware() {
    struct stat info;
    if (!sd.ready || stat("/sdcard/firmware.bin", &info) != 0) return;
    FILE* file = fopen("/sdcard/firmware.bin", "rb");
    if (!file) return;
    log("OTA: installing firmware.bin from SD");
    esp_err_t err = installFirmware(file, info.st_size);
    fclose(file);
    if (err == ESP_OK) {
        remove("/sdcard/firmware.bin");
        log("OTA: SD update complete; restarting");
        esp_restart();
    } else {
        remove("/sdcard/firmware.failed.bin");
        rename("/sdcard/firmware.bin", "/sdcard/firmware.failed.bin");
        log("OTA: SD update rejected: %s", esp_err_to_name(err));
    }
}
}
