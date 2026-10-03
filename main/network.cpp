#include "app.hpp"
#include "wifi_retry.hpp"
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "mdns.h"

namespace pov {
namespace {
esp_netif_t* station;
QueueHandle_t requests;
struct NetworkRequest {
    char ssid[33] = {}, password[64] = {}, hostname[64] = {};
    bool manual = false;
};
std::atomic<bool> staStarted{false}, staLinked{false}, hasIp{false}, connecting{false}, enabled{false};
std::atomic<unsigned> apClients{0}, apJoins{0}, apDisconnects{0}, staDisconnects{0}, gotIpCount{0}, attempts{0};
std::atomic<unsigned> apReason{0}, staReason{0}, lastApDisconnectMs{0};
std::atomic<int> staDisconnectRssi{0};
std::atomic<uint32_t> stationAddress{0}, eventStackFreeMin{UINT32_MAX};
SemaphoreHandle_t scanMutex;
std::atomic<bool> scanQueued{false}, scanBusy{false};
struct ScanNetwork { std::string ssid; int rssi; unsigned channel; wifi_auth_mode_t auth; };
std::vector<ScanNetwork> scanNetworks;
std::string scanState = "idle", scanError;
bool scanTruncated = false;
enum class ScopeRadio { Active, Stopping, Stopped, Starting };
std::atomic<ScopeRadio> scopeRadio{ScopeRadio::Active};
std::atomic<bool> scopeQuietRequested{false};
std::atomic<esp_err_t> scopeRadioError{ESP_OK};

const char* authName(wifi_auth_mode_t auth) {
    switch (auth) {
    case WIFI_AUTH_OPEN: return "Open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "Enterprise";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    case WIFI_AUTH_OWE: return "Enhanced open";
    default: return "Other security";
    }
}
void performScan() {
    // Only the network task changes radio state. The HTTP task returns a job
    // immediately; neither the display lock nor the web server waits here.
    esp_err_t err = connecting.load() ? ESP_ERR_WIFI_STATE : ESP_OK;
    wifi_mode_t original = WIFI_MODE_NULL;
    if (err == ESP_OK) err = esp_wifi_get_mode(&original);
    if (err == ESP_OK && original == WIFI_MODE_AP) err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    wifi_scan_config_t cfg = {};
    cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    cfg.scan_time.active.min = 20;
    cfg.scan_time.active.max = 80;
    cfg.home_chan_dwell_time = 60;
    const bool scanAttempted = err == ESP_OK;
    if (scanAttempted) err = esp_wifi_scan_start(&cfg, true);
    constexpr uint16_t limit = 32;
    uint16_t found = 0, count = limit;
    wifi_ap_record_t* records = nullptr;
    if (err == ESP_OK) err = esp_wifi_scan_get_ap_num(&found);
    if (err == ESP_OK && found) {
        records = static_cast<wifi_ap_record_t*>(calloc(limit, sizeof(wifi_ap_record_t)));
        err = records ? esp_wifi_scan_get_ap_records(&count, records) : ESP_ERR_NO_MEM;
    } else count = 0;
    // Release our scan memory on every path, but do not touch the driver's
    // connection scan if it rejected this request as busy.
    if (scanAttempted && err != ESP_ERR_WIFI_STATE) {
        if (err != ESP_OK) esp_wifi_scan_stop();
        if (err != ESP_OK || !found) esp_wifi_clear_ap_list();
    }
    if (original == WIFI_MODE_AP) {
        const esp_err_t restored = esp_wifi_set_mode(WIFI_MODE_AP);
        if (err == ESP_OK) err = restored;
    }
    xSemaphoreTake(scanMutex, portMAX_DELAY);
    scanNetworks.clear();
    if (err == ESP_OK) {
        for (unsigned i = 0; i < count; ++i) {
            records[i].ssid[32] = 0;
            const std::string ssid(reinterpret_cast<const char*>(records[i].ssid));
            if (ssid.empty()) continue;
            // IDF returns strongest first; keep one entry per SSID/security.
            const bool duplicate = std::any_of(scanNetworks.begin(), scanNetworks.end(), [&](const ScanNetwork& n) {
                return n.ssid == ssid && n.auth == records[i].authmode;
            });
            if (!duplicate) scanNetworks.push_back({ssid, records[i].rssi, records[i].primary, records[i].authmode});
        }
    }
    scanTruncated = found > limit;
    scanError = err == ESP_OK ? "" : err == ESP_ERR_WIFI_STATE ?
        "Router connection is in progress. Wait a moment and scan again." : esp_err_to_name(err);
    scanState = err == ESP_OK ? "complete" : "failed";
    scanBusy = false;
    xSemaphoreGive(scanMutex);
    free(records);
    log("Wi-Fi scan: %s, %u access points", esp_err_to_name(err), found);
}

void wifiEvent(void*, esp_event_base_t base, int32_t event, void* data) {
    // This runs on IDF's shared sys_evt stack. Copy event data only: formatting,
    // logging, locks and driver queries belong on the network task. Build 18's
    // saved crash showed vApplicationStackOverflowHook on this event task.
    if (base == WIFI_EVENT) {
        switch (event) {
        case WIFI_EVENT_STA_START: staStarted = true; break;
        case WIFI_EVENT_STA_STOP:
            staStarted = false; staLinked = false; hasIp = false; connecting = false;
            break;
        case WIFI_EVENT_STA_CONNECTED: staLinked = true; connecting = false; break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            const auto* info = static_cast<wifi_event_sta_disconnected_t*>(data);
            staLinked = false; hasIp = false; connecting = false;
            staReason = info->reason; staDisconnectRssi = info->rssi; ++staDisconnects;
            break;
        }
        case WIFI_EVENT_AP_START:
        case WIFI_EVENT_AP_STOP: apClients = 0; break;
        case WIFI_EVENT_AP_STACONNECTED:
            ++apClients; ++apJoins; break;
        case WIFI_EVENT_AP_STADISCONNECTED: {
            const auto* info = static_cast<wifi_event_ap_stadisconnected_t*>(data);
            apReason = info->reason;
            lastApDisconnectMs = esp_timer_get_time() / 1000;
            // Only these serialized callbacks write apClients; guard a late
            // disconnect delivered after AP_STOP so the count cannot underflow.
            if (apClients.load()) --apClients;
            ++apDisconnects;
            break;
        }
        default: break;
        }
    } else if (base == IP_EVENT && event == IP_EVENT_STA_GOT_IP) {
        const auto* got = static_cast<ip_event_got_ip_t*>(data);
        stationAddress = got->ip_info.ip.addr;
        hasIp = true; ++gotIpCount;
    } else if (base == IP_EVENT && event == IP_EVENT_STA_LOST_IP) {
        hasIp = false;
    }
    // IDF reports stack high-water marks in bytes. Reading this task's minimum
    // also includes deeper calls made by the other default-loop handlers.
    eventStackFreeMin = std::min(eventStackFreeMin.load(),
        static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr)));
}
void requestNetwork(bool manual) {
    NetworkRequest request;
    memcpy(request.ssid, config.ssid.data(), std::min(config.ssid.size(), sizeof(request.ssid) - 1));
    memcpy(request.password, config.password.data(), std::min(config.password.size(), sizeof(request.password) - 1));
    memcpy(request.hostname, config.hostname.data(), std::min(config.hostname.size(), sizeof(request.hostname) - 1));
    request.manual = manual;
    xQueueOverwrite(requests, &request);
}
void networkTask(void*) {
    NetworkRequest active;
    WifiRetry retry;
    unsigned seenDisconnects = 0, seenIp = 0;
    unsigned loggedJoins = 0, loggedApDisconnects = 0, loggedStaDisconnects = 0, loggedIp = 0;
    int64_t quietDeadline = 0;
    while (true) {
        // This task owns all radio changes. No scans, configuration changes or
        // connection retries may run between the stop acknowledgement and resume.
        if (scopeQuietRequested.load() || scopeRadio.load() != ScopeRadio::Active) {
            if (scopeRadio.load() == ScopeRadio::Active) {
                scopeRadio = ScopeRadio::Stopping;
                const esp_err_t err = esp_wifi_stop();
                scopeRadioError = err;
                if (err == ESP_OK) {
                    quietDeadline = esp_timer_get_time() + 90000000;
                    scopeRadio = ScopeRadio::Stopped;
                } else {
                    scopeQuietRequested = false;
                    scopeRadio = ScopeRadio::Active;
                }
            } else if (scopeRadio.load() == ScopeRadio::Stopped &&
                       scopeQuietRequested.load() && esp_timer_get_time() < quietDeadline) {
                vTaskDelay(pdMS_TO_TICKS(20));
            } else {
                // The lease also restores connectivity if the capture worker
                // never reaches cleanup. Keep retrying a failed restart.
                scopeQuietRequested = false;
                scopeRadio = ScopeRadio::Starting;
                const esp_err_t err = esp_wifi_start();
                scopeRadioError = err;
                if (err == ESP_OK) {
                    esp_wifi_set_ps(WIFI_PS_NONE);
                    seenDisconnects = staDisconnects.load(); seenIp = gotIpCount.load();
                    retry.reset(enabled.load(), true, esp_timer_get_time());
                    scopeRadio = ScopeRadio::Active;
                } else vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }
        NetworkRequest request;
        if (xQueueReceive(requests, &request, pdMS_TO_TICKS(250)) == pdTRUE) {
            esp_netif_set_hostname(station, request.hostname);
            mdns_hostname_set(request.hostname);
            const bool changed = strcmp(active.ssid, request.ssid) || strcmp(active.password, request.password) ||
                (request.ssid[0] && !enabled.load());
            active = request;
            // Saving just a hostname must not drop a healthy router connection.
            if (changed) {
                enabled = false; hasIp = false; staLinked = false; connecting = false;
                if (staStarted.load()) esp_wifi_disconnect();
                esp_err_t err = esp_wifi_set_mode(request.ssid[0] ? WIFI_MODE_APSTA : WIFI_MODE_AP);
                if (err == ESP_OK && request.ssid[0]) {
                    wifi_config_t cfg = {};
                    memcpy(cfg.sta.ssid, request.ssid, std::min(strlen(request.ssid), sizeof(cfg.sta.ssid)));
                    memcpy(cfg.sta.password, request.password, std::min(strlen(request.password), sizeof(cfg.sta.password)));
                    cfg.sta.pmf_cfg.capable = true;
                    err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
                    if (err == ESP_OK) err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
                }
                enabled = request.ssid[0] && err == ESP_OK;
                if (err != ESP_OK) log("Wi-Fi configuration failed: %s", esp_err_to_name(err));
            }
            retry.reset(enabled.load(), request.manual, esp_timer_get_time());
            if (staLinked.load()) retry.connected();
            seenDisconnects = staDisconnects.load(); seenIp = gotIpCount.load();
        }
        // Coalesce bursts to the latest event without allocating or blocking
        // in sys_evt. These counters are separate from the retry state below.
        const unsigned joins = apJoins.load(), apDrops = apDisconnects.load();
        const unsigned staDrops = staDisconnects.load(), ips = gotIpCount.load();
        if (loggedJoins != joins) {
            loggedJoins = joins;
            log("POV-Spinner client joined; %u connected", apClients.load());
        }
        if (loggedApDisconnects != apDrops) {
            loggedApDisconnects = apDrops;
            log("POV-Spinner client disconnected: reason %u; uptime %u ms; %u remaining",
                apReason.load(), lastApDisconnectMs.load(), apClients.load());
        }
        if (loggedStaDisconnects != staDrops) {
            loggedStaDisconnects = staDrops;
            log("Wi-Fi router disconnected: reason %u, RSSI %d dBm", staReason.load(), staDisconnectRssi.load());
        }
        if (loggedIp != ips) {
            loggedIp = ips;
            esp_ip4_addr_t address = {}; address.addr = stationAddress.load();
            log("Wi-Fi station connected: " IPSTR, IP2STR(&address));
        }
        const int64_t now = esp_timer_get_time();
        if (seenIp != gotIpCount.load()) { seenIp = gotIpCount.load(); retry.connected(); }
        if (seenDisconnects != staDisconnects.load()) {
            seenDisconnects = staDisconnects.load(); retry.failed(now);
            if (retry.attempts >= WifiRetry::Limit)
                log("Wi-Fi router retries paused after %u attempts; POV-Spinner remains available", retry.attempts);
        }
        if (scanQueued.exchange(false)) performScan();
        if (!scanBusy.load() && staStarted.load() && !staLinked.load() && !connecting.load() && retry.ready(now, apClients.load())) {
            retry.started(); connecting = true;
            const esp_err_t err = esp_wifi_connect();
            log("Wi-Fi router attempt %u/%u: %s", retry.attempts, WifiRetry::Limit, esp_err_to_name(err));
            if (err != ESP_OK) { connecting = false; retry.failed(now); }
        }
        attempts = retry.attempts;
    }
}
}
bool wifiQuietForScope() {
    return scopeQuietRequested.load() && scopeRadio.load() == ScopeRadio::Stopped;
}
bool pauseWifiForScope(std::string& error) {
    if (scopeRadio.load() != ScopeRadio::Active || scopeQuietRequested.load()) {
        error = "Wi-Fi is already pausing or restoring"; return false;
    }
    scopeRadioError = ESP_OK;
    scopeQuietRequested = true;
    const int64_t deadline = esp_timer_get_time() + 5000000;
    while (esp_timer_get_time() < deadline) {
        if (wifiQuietForScope()) return true;
        if (scopeRadioError.load() != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    scopeQuietRequested = false;
    error = "Cannot stop Wi-Fi for capture: " + std::string(esp_err_to_name(
        scopeRadioError.load() == ESP_OK ? ESP_ERR_TIMEOUT : scopeRadioError.load()));
    return false;
}
bool resumeWifiAfterScope(std::string& error) {
    scopeQuietRequested = false;
    const int64_t deadline = esp_timer_get_time() + 10000000;
    while (esp_timer_get_time() < deadline) {
        if (scopeRadio.load() == ScopeRadio::Active) return true;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    error = "Wi-Fi restart is still retrying: " + std::string(esp_err_to_name(
        scopeRadioError.load() == ESP_OK ? ESP_ERR_TIMEOUT : scopeRadioError.load()));
    return false;
}
bool startWifiScan(std::string& error) {
    xSemaphoreTake(scanMutex, portMAX_DELAY);
    if (scopeQuietRequested.load() || scopeRadio.load() != ScopeRadio::Active) {
        error = "Wi-Fi is paused for the SD scope";
        xSemaphoreGive(scanMutex); return false;
    }
    if (scanBusy.load() || connecting.load()) {
        error = scanBusy.load() ? "A Wi-Fi scan is already running" : "Router connection is in progress. Try scanning again shortly.";
        xSemaphoreGive(scanMutex); return false;
    }
    scanNetworks.clear(); scanError.clear(); scanTruncated = false;
    scanState = "scanning"; scanBusy = true; scanQueued = true;
    xSemaphoreGive(scanMutex);
    return true;
}
cJSON* wifiScanJson() {
    xSemaphoreTake(scanMutex, portMAX_DELAY);
    cJSON* j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "running", scanBusy.load());
    cJSON_AddStringToObject(j, "state", scanState.c_str());
    cJSON_AddStringToObject(j, "error", scanError.c_str());
    cJSON_AddBoolToObject(j, "truncated", scanTruncated);
    cJSON* networks = cJSON_AddArrayToObject(j, "networks");
    for (const auto& n : scanNetworks) {
        cJSON* entry = cJSON_CreateObject();
        cJSON_AddStringToObject(entry, "ssid", n.ssid.c_str());
        cJSON_AddNumberToObject(entry, "rssi", n.rssi);
        cJSON_AddNumberToObject(entry, "channel", n.channel);
        cJSON_AddStringToObject(entry, "security", authName(n.auth));
        cJSON_AddBoolToObject(entry, "open", n.auth == WIFI_AUTH_OPEN);
        cJSON_AddItemToArray(networks, entry);
    }
    xSemaphoreGive(scanMutex);
    return j;
}
std::string stationIp() {
    esp_netif_ip_info_t info = {};
    if (!hasIp.load() || !station || esp_netif_get_ip_info(station, &info) != ESP_OK || !info.ip.addr) return "";
    char value[20]; snprintf(value, sizeof(value), IPSTR, IP2STR(&info.ip)); return value;
}
void addNetworkStatus(cJSON* j) {
    const bool configured = enabled.load();
    const bool paused = configured && !staLinked.load() && !connecting.load() &&
        (apClients.load() || attempts.load() >= WifiRetry::Limit);
    cJSON_AddStringToObject(j, "mode", configured ? "AP+STA" : "AP only");
    cJSON_AddBoolToObject(j, "stationConnected", hasIp.load());
    cJSON_AddBoolToObject(j, "stationConnecting", connecting.load());
    cJSON_AddBoolToObject(j, "scanning", scanBusy.load());
    cJSON_AddBoolToObject(j, "scopeRadioPaused", scopeRadio.load() != ScopeRadio::Active);
    cJSON_AddBoolToObject(j, "retriesPaused", paused);
    cJSON_AddNumberToObject(j, "routerAttempts", attempts.load());
    cJSON_AddNumberToObject(j, "apClients", apClients.load());
    cJSON_AddNumberToObject(j, "apDisconnects", apDisconnects.load());
    cJSON_AddNumberToObject(j, "lastApDisconnectReason", apReason.load());
    cJSON_AddNumberToObject(j, "lastApDisconnect_ms", lastApDisconnectMs.load());
    cJSON_AddNumberToObject(j, "routerDisconnects", staDisconnects.load());
    cJSON_AddNumberToObject(j, "lastRouterDisconnectReason", staReason.load());
    cJSON_AddNumberToObject(j, "uptime_ms", esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(j, "resetReason", esp_reset_reason());
    cJSON_AddBoolToObject(j, "brownoutReset", esp_reset_reason() == ESP_RST_BROWNOUT);
    const uint32_t stackFree = eventStackFreeMin.load();
    if (stackFree != UINT32_MAX) cJSON_AddNumberToObject(j, "eventTaskMinFreeStack_bytes", stackFree);
    uint8_t channel = 0; wifi_second_chan_t second;
    if (esp_wifi_get_channel(&channel, &second) == ESP_OK) cJSON_AddNumberToObject(j, "channel", channel);
    wifi_bandwidth_t bandwidth;
    if (esp_wifi_get_bandwidth(WIFI_IF_AP, &bandwidth) == ESP_OK)
        cJSON_AddNumberToObject(j, "apBandwidthMHz", bandwidth == WIFI_BW_HT20 ? 20 : 40);
    wifi_ap_record_t router = {};
    if (staLinked.load() && esp_wifi_sta_get_ap_info(&router) == ESP_OK)
        cJSON_AddNumberToObject(j, "routerRssi", router.rssi);
}
void reconnectNetwork() { requestNetwork(true); }
void startNetwork() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    station = esp_netif_create_default_wifi_sta();
    requests = xQueueCreate(1, sizeof(NetworkRequest));
    scanMutex = xSemaphoreCreateMutex();
    configASSERT(requests && scanMutex);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEvent, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEvent, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, wifiEvent, nullptr));
    // Start with a fixed AP. Enable the station only when a router is configured.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap = {};
    strcpy(reinterpret_cast<char*>(ap.ap.ssid), AccessPointName);
    strcpy(reinterpret_cast<char*>(ap.ap.password), AccessPointPassword);
    ap.ap.channel = 1; ap.ap.max_connection = 4; ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    // Keep AP and station on a single 20 MHz channel. A setup/control link
    // does not need HT40; narrower operation avoids secondary-channel
    // negotiation when the AP and router share this radio.
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(mdns_init());
    mdns_instance_name_set("LPOVXLM spinner");
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    requestNetwork(false);
    configASSERT(xTaskCreatePinnedToCore(networkTask, "network", 4096, nullptr, 2, nullptr, 0) == pdPASS);
    log("Wi-Fi AP: POV-Spinner; hostname: %s.local", config.hostname.c_str());
}
}
