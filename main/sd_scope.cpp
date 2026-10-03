#include "app.hpp"
#include "sd_scope.hpp"
#include <algorithm>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

namespace pov {
bool restoreSdScopePins(std::string& error) {
    const int pins[] = {BoardPins::SdClk, BoardPins::SdCmd, BoardPins::SdD0,
                        BoardPins::SdD1, BoardPins::SdD2, BoardPins::SdD3};
    esp_err_t first = ESP_OK;
    for (int pin : pins) {
        const auto gpio = static_cast<gpio_num_t>(pin);
        const auto rtc = rtc_gpio_deinit(gpio);
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = pin == BoardPins::SdClk ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
        const auto digital = gpio_config(&cfg);
        if (first == ESP_OK) first = rtc != ESP_OK ? rtc : digital;
    }
    if (first == ESP_OK) return true;
    error = "Cannot restore SD pins: " + std::string(esp_err_to_name(first));
    return false;
}

bool captureSdScope(const SdScopeOptions& options, std::atomic<bool>& cancel,
                    SdScopeResult& result, std::string& error) {
    result.pin = options.pin;
    adc_unit_t unit;
    adc_channel_t channel;
    adc_oneshot_unit_handle_t adc = nullptr;
    adc_cali_handle_t calibration = nullptr;
    esp_err_t err = adc_oneshot_io_to_channel(options.pin, &unit, &channel);
    if (err == ESP_OK) {
        result.adcUnit = unsigned(unit) + 1;
        adc_oneshot_unit_init_cfg_t init = {};
        init.unit_id = unit;
        err = adc_oneshot_new_unit(&init, &adc);
    }
    if (err == ESP_OK) {
        adc_oneshot_chan_cfg_t cfg = {};
        cfg.bitwidth = ADC_BITWIDTH_12;
        cfg.atten = ADC_ATTEN_DB_12;
        err = adc_oneshot_config_channel(adc, channel, &cfg);
    }
    if (err == ESP_OK) {
        adc_cali_curve_fitting_config_t cfg = {};
        cfg.unit_id = unit; cfg.chan = channel;
        cfg.atten = ADC_ATTEN_DB_12; cfg.bitwidth = ADC_BITWIDTH_12;
        const auto cal = adc_cali_create_scheme_curve_fitting(&cfg, &calibration);
        result.calibrated = cal == ESP_OK;
        if (cal != ESP_OK) result.calibrationError = esp_err_to_name(cal);

        result.points.reserve(SdScopeSamples);
        vTaskDelay(pdMS_TO_TICKS(2));
        // A discarded conversion settles the sample capacitor after switching channels.
        int discarded;
        adc_oneshot_read(adc, channel, &discarded);
        const int64_t start = esp_timer_get_time();
        result.startedUs = start;
        int64_t next = start, lastYield = start;
        unsigned valid = 0;
        for (unsigned i = 0; i < SdScopeSamples && !cancel.load(); ++i) {
            // Yield during long waits; never fabricate uniformly spaced timestamps.
            int64_t now = esp_timer_get_time();
            while (next - now > 2000 && !cancel.load()) {
                vTaskDelay(1); now = esp_timer_get_time(); lastYield = now;
            }
            if (cancel.load()) break;
            if (next > now) esp_rom_delay_us(static_cast<uint32_t>(next - now));
            const int64_t before = esp_timer_get_time();
            SdScopePoint point;
            int raw;
            const auto reading = adc_oneshot_read(adc, channel, &raw);
            const int64_t after = esp_timer_get_time();
            point.timeUs = static_cast<uint32_t>((before + after) / 2 - start);
            if (reading == ESP_OK) { point.raw = raw; point.upperLimit = raw >= 4090; ++valid; }
            else result.readError = esp_err_to_name(reading);
            result.points.push_back(point);
            // Skip elapsed deadlines instead of producing a burst of catch-up samples.
            next = std::max(next + options.intervalUs, after + options.intervalUs);
            if (after - lastYield >= 2000) { vTaskDelay(1); lastYield = esp_timer_get_time(); }
        }
        result.cancelled = cancel.load();
        // Convert after acquisition so calibration work does not distort sample timing.
        if (calibration) {
            for (auto& point : result.points) if (point.raw >= 0) {
                int mv;
                const auto converted = adc_cali_raw_to_voltage(calibration, point.raw, &mv);
                if (converted == ESP_OK) {
                    // The ADC2 calibration polynomial can extrapolate to almost 5 V
                    // on saturated 3.3 V inputs. Preserve the raw sample and range
                    // flag, but never publish that extrapolation as measured voltage.
                    point.upperLimit |= mv >= 3100;
                    if (!point.upperLimit && mv >= 0) point.millivolts = mv;
                }
                else result.calibrationError = esp_err_to_name(converted);
            }
        }
        if (!valid && !result.cancelled) error = "ADC returned no valid samples: " + result.readError;
    } else error = "Cannot configure ADC: " + std::string(esp_err_to_name(err));

    if (calibration) adc_cali_delete_scheme_curve_fitting(calibration);
    if (adc) {
        const auto released = adc_oneshot_del_unit(adc);
        if (released != ESP_OK) error += "; ADC cleanup failed: " + std::string(esp_err_to_name(released));
    }
    return error.empty() && !result.cancelled;
}
}
