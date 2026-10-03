#include "app.hpp"
#include "radial_brightness.hpp"
#include "strip_speed.hpp"
#include "playback_timing.hpp"
#include <algorithm>
#include <cstring>
#include <new>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"

namespace pov {
Output output;
static_assert(BoardPins::InitialArms == MaxArms, "Update native geometry before adding arms");
struct DmaOutput {
    esp_lcd_i80_bus_handle_t bus = nullptr;
    esp_lcd_panel_io_handle_t io = nullptr;
    SemaphoreHandle_t done = nullptr;
    uint8_t* buffer = nullptr;
    uint8_t* black = nullptr;
    uint8_t* pulse = nullptr;
    unsigned pulseColorFrames = 1;
    int64_t completedAt = 0;
    bool inFlight = false;
};
namespace {
esp_err_t parkGpio() {
    // Configure the pad mux as well as the matrix and direction, both on a
    // cold boot and when returning from LCD output. Direction alone does not
    // select the GPIO pad function. Enable input too for diagnostic readback.
    gpio_config_t pins = {};
    pins.pin_bit_mask = 1ULL << BoardPins::Clock;
    gpio_set_level(static_cast<gpio_num_t>(BoardPins::Clock), 0);
    for (int pin : BoardPins::OutputData) {
        pins.pin_bit_mask |= 1ULL << pin;
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
    }
    pins.mode = GPIO_MODE_INPUT_OUTPUT;
    esp_err_t err = gpio_config(&pins);
    if (err != ESP_OK) return err;
    esp_rom_gpio_connect_out_signal(BoardPins::Clock, SIG_GPIO_OUT_IDX, false, false);
    for (int pin : BoardPins::OutputData)
        esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    return ESP_OK;
}
bool IRAM_ATTR dmaDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void* context) {
    auto* dma = static_cast<DmaOutput*>(context);
    dma->completedAt = esp_timer_get_time();
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(dma->done, &wake);
    return wake == pdTRUE;
}
}
bool Output::disableDma() {
    if (dma_) {
        // Never delete driver queues or reuse memory while DMA still owns it.
        // After a timeout retain resources until completion (or a reboot).
        if (dma_->inFlight && xSemaphoreTake(dma_->done, 0) != pdTRUE) return false;
        if (dma_->io) esp_lcd_panel_io_del(dma_->io);
        if (dma_->bus) esp_lcd_del_i80_bus(dma_->bus);
        if (dma_->done) vSemaphoreDelete(dma_->done);
        free(dma_->buffer);
        free(dma_->black);
        free(dma_->pulse);
        dma_->~DmaOutput();
        free(dma_);
        dma_ = nullptr;
    }
    const esp_err_t err = parkGpio();
    if (err != ESP_OK) { error_ = err; fault_ = true; return false; }
    fault_ = false;
    return true;
}
bool Output::enableDma(uint32_t clockHz) {
    if (!rgb_ || !validStripClock(clockHz)) {
        error_ = ESP_ERR_INVALID_ARG; return false;
    }
    if (!disableDma()) { error_ = ESP_ERR_INVALID_STATE; return false; }
    void* storage = heap_caps_malloc(sizeof(DmaOutput), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!storage) { error_ = ESP_ERR_NO_MEM; return false; }
    dma_ = new(storage) DmaOutput;
    dma_->done = xSemaphoreCreateBinary();
    if (!dma_->done) { error_ = ESP_ERR_NO_MEM; disableDma(); return false; }
    esp_lcd_i80_bus_config_t bus = {};
    bus.clk_src = LCD_CLK_SRC_PLL160M;
    bus.wr_gpio_num = BoardPins::Clock;
    // ESP-IDF 5.3 requires eight connected data pins and a DC pin. Use only
    // existing optional LED outputs for these extra signals, always at zero.
    for (unsigned lane = 0; lane < MaxArms; ++lane) bus.data_gpio_nums[lane] = BoardPins::ArmData[lane];
    constexpr int parked[] = {BoardPins::OutputData[1], BoardPins::OutputData[2],
                              BoardPins::OutputData[3], BoardPins::OutputData[5]};
    for (unsigned lane = 0; lane < 4; ++lane) bus.data_gpio_nums[lane + 4] = parked[lane];
    bus.dc_gpio_num = BoardPins::OutputData[6]; // GPIO40, all DC levels zero
    bus.bus_width = 8;
    const size_t frameClocks = SharedClockProtocol::frameBytes(pixels_, protocol_) * 8;
    bus.max_transfer_bytes = frameClocks * (MaximumPulseColorFrames + 1);
    bus.dma_burst_size = 16;
    error_ = esp_lcd_new_i80_bus(&bus, &dma_->bus);
    if (error_ == ESP_OK) {
        esp_lcd_panel_io_i80_config_t io = {};
        io.cs_gpio_num = -1;
        io.pclk_hz = clockHz;
        io.trans_queue_depth = 1;
        io.on_color_trans_done = dmaDone;
        io.user_ctx = dma_;
        io.lcd_cmd_bits = 8;
        io.lcd_param_bits = 8;
        io.flags.pclk_idle_low = true;
        io.flags.pclk_active_neg = false; // LEDs sample on the rising edge.
        // No byte/bit swapping: each byte is one set of parallel data levels.
        error_ = esp_lcd_new_panel_io_i80(dma_->bus, &io, &dma_->io);
    }
    if (error_ == ESP_OK) {
        dma_->buffer = static_cast<uint8_t*>(esp_lcd_i80_alloc_draw_buffer(
            dma_->io, frameClocks, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        dma_->black = static_cast<uint8_t*>(esp_lcd_i80_alloc_draw_buffer(
            dma_->io, frameClocks, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        if (!dma_->buffer || !dma_->black) error_ = ESP_ERR_NO_MEM;
        else SharedClockProtocol::packBlackFourLanes(pixels_, dma_->black, frameClocks, protocol_);
    }
    if (error_ != ESP_OK) { disableDma(); return false; }
    stats = {};
    return true;
}
bool Output::begin(unsigned pixels) {
    if (!pixels || pixels > MaxPixels) return false;
    auto* next = static_cast<uint8_t*>(heap_caps_calloc(MaxArms * pixels, 3, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!next) return false;
    if (!disableDma()) { free(next); return false; }
    free(rgb_); rgb_ = next; pixels_ = pixels; stats = {};
    protocol_ = static_cast<SharedClockProtocol::Protocol>(config.ledProtocol);
    return show(0);
}
void Output::pixel(unsigned arm, unsigned index, uint8_t r, uint8_t g, uint8_t b, bool applyCenterFade, bool tipFirst) {
    if (!rgb_ || arm >= MaxArms || index >= pixels_) return;
    index = BoardPins::imagePixelToWire(arm, index, pixels_, tipFirst);
    // Shape physical brightness independently of the sequence's pixel order.
    if (applyCenterFade && config.centerDimming) {
        const uint16_t gain = radialGains_.gain(index, pixels_, config.brightness,
            config.centerBrightness, true, BoardPins::ArmInputAtHub[arm]);
        r = (unsigned(r) * gain + 128) >> 8;
        g = (unsigned(g) * gain + 128) >> 8;
        b = (unsigned(b) * gain + 128) >> 8;
    }
    auto* pixel = rgb_ + (arm * pixels_ + index) * 3;
    pixel[0] = r; pixel[1] = g; pixel[2] = b;
    hasColor_ |= r || g || b;
}
void Output::row(unsigned arm, const uint8_t* colors, bool tipFirst) {
    if (!colors) return;
    for (unsigned i = 0; i < pixels_; ++i)
        pixel(arm, i, colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2], true, tipFirst);
}
void Output::clear() { if (rgb_) memset(rgb_, 0, MaxArms * pixels_ * 3); hasColor_ = false; }
bool Output::signalLevel(int pin, bool high) {
    bool allowed = pin == BoardPins::Clock;
    for (int data : BoardPins::ArmData) allowed |= pin == data;
    if (!allowed || !ready() || dma_) { error_ = ESP_ERR_INVALID_STATE; return false; }
    error_ = gpio_set_level(static_cast<gpio_num_t>(pin), high);
    return error_ == ESP_OK;
}
bool Output::preparePulse(unsigned colorFrames) {
    if (!dma_) { error_ = ESP_ERR_INVALID_STATE; return false; }
    if (!colorFrames || colorFrames > MaximumPulseColorFrames) { error_ = ESP_ERR_INVALID_ARG; return false; }
    const size_t frame = SharedClockProtocol::frameBytes(pixels_, protocol_) * 8;
    if (!dma_->pulse) dma_->pulse = static_cast<uint8_t*>(esp_lcd_i80_alloc_draw_buffer(
        dma_->io, frame * (MaximumPulseColorFrames + 1), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    if (!dma_->pulse) { error_ = ESP_ERR_NO_MEM; return false; }
    // Repeated identical color frames extend the pulse; black is still part
    // of the same hardware transfer, without a software blanking delay.
    dma_->pulseColorFrames = colorFrames;
    memcpy(dma_->pulse + frame * colorFrames, dma_->black, frame);
    return true;
}
bool Output::show(uint8_t brightness, bool pulse, int64_t deadline) {
    if (!ready()) { error_ = ESP_ERR_INVALID_STATE; return false; }
    if (pulse && (!dma_ || !dma_->pulse)) { error_ = ESP_ERR_INVALID_STATE; return false; }
    const size_t bytes = SharedClockProtocol::frameBytes(pixels_, protocol_);
    size_t frames = 1;
    const int64_t start = esp_timer_get_time();
    stats.lastPulse = stats.lastSkipped = false;
    stats.lastPackUs = 0; stats.lastCompletionUs = 0;
    if (dma_) {
        const bool black = !hasColor_ || !brightness;
        frames = pulse && !black ? dma_->pulseColorFrames + 1 : 1;
        uint8_t* destination = frames > 1 ? dma_->pulse : dma_->buffer;
        if (!black) SharedClockProtocol::packFourLanes(rgb_, pixels_, brightness, destination, bytes * 8, protocol_);
        for (size_t frame = 1; frame + 1 < frames; ++frame)
            memcpy(destination + frame * bytes * 8, destination, bytes * 8);
        const int64_t submit = esp_timer_get_time();
        stats.lastPackUs = submit - start;
        // Reject a late pulse after packing, before any colored data is sent.
        // The previous pulse has already returned the LEDs to black.
        if (frames > 1 && !preparedPulseFits(submit, deadline, bytes * 8, dma_->pulseColorFrames)) {
            stats.lastSkipped = true;
            stats.lastUs = submit - start;
            stats.bits = 0;
            return true;
        }
        dma_->inFlight = true;
        // -1 suppresses the LCD command phase; only the LED stream is clocked.
        error_ = esp_lcd_panel_io_tx_color(dma_->io, -1, black ? dma_->black : destination, bytes * 8 * frames);
        if (error_ != ESP_OK) { dma_->inFlight = false; return false; }
        if (xSemaphoreTake(dma_->done, pdMS_TO_TICKS(250)) != pdTRUE) {
            error_ = ESP_ERR_TIMEOUT; fault_ = true;
            parkGpio(); // Disconnect clock; retain the still-owned DMA buffer.
            return false;
        }
        dma_->inFlight = false;
        stats.lastCompletionUs = dma_->completedAt - submit;
        stats.lastPulse = frames > 1;
    } else {
        uint32_t masksLow[MaxArms] = {}, masksHigh[MaxArms] = {};
        uint32_t allLow = 0, allHigh = 0;
        for (unsigned lane = 0; lane < MaxArms; ++lane) {
            int pin = BoardPins::ArmData[lane];
            if (pin < 32) allLow |= masksLow[lane] = 1UL << pin;
            else allHigh |= masksHigh[lane] = 1UL << (pin - 32);
        }
        // GPIO42 is bit 10 of the upper register bank. Keep both edges on
        // that bank; GPIO1 is assigned to motor speed PWM.
        constexpr uint32_t clockMask = 1UL << (BoardPins::Clock % 32);
        for (size_t byte = 0; byte < bytes; ++byte) {
            uint8_t values[MaxArms];
            for (unsigned lane = 0; lane < MaxArms; ++lane)
                values[lane] = SharedClockProtocol::wireByte(rgb_ + lane * pixels_ * 3, pixels_, byte, brightness, false, protocol_);
            for (uint8_t bit = 0x80; bit; bit >>= 1) {
                uint32_t low = 0, high = 0;
                for (unsigned lane = 0; lane < MaxArms; ++lane)
                    if (values[lane] & bit) { low |= masksLow[lane]; high |= masksHigh[lane]; }
                GPIO.out_w1tc = allLow;
                GPIO.out1_w1tc.val = allHigh;
                GPIO.out_w1ts = low;
                GPIO.out1_w1ts.val = high;
                if constexpr (BoardPins::Clock < 32) GPIO.out_w1ts = clockMask;
                else GPIO.out1_w1ts.val = clockMask;
                asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;");
                if constexpr (BoardPins::Clock < 32) GPIO.out_w1tc = clockMask;
                else GPIO.out1_w1tc.val = clockMask;
            }
        }
        GPIO.out_w1tc = allLow; GPIO.out1_w1tc.val = allHigh;
    }
    stats.lastUs = esp_timer_get_time() - start;
    stats.minUs = stats.count ? std::min(stats.minUs, stats.lastUs) : stats.lastUs;
    stats.maxUs = std::max(stats.maxUs, stats.lastUs);
    stats.bits = bytes * 8 * frames;
    stats.totalUs += stats.lastUs; ++stats.count;
    if (hasColor_ && brightness) ++stats.colorCount;
    else ++stats.blackCount;
    stats.totalPackUs += stats.lastPackUs;
    stats.totalCompletionUs += stats.lastCompletionUs;
    stats.maxCompletionUs = std::max(stats.maxCompletionUs, stats.lastCompletionUs);
    error_ = ESP_OK;
    return true;
}
}
