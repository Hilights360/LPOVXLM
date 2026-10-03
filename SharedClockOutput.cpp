#include "SharedClockOutput.h"
#include "SharedClockProtocol.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "soc/gpio_struct.h"
#include <string.h>

namespace {
bool usablePin(int pin) {
  return pin >= 0 && GPIO_IS_VALID_OUTPUT_GPIO(pin) &&
         !(pin >= 26 && pin <= 37); // Flash and octal PSRAM.
}
}

bool SharedClockOutput::begin(int clockPin, const int* dataPins,
                              uint8_t lanes, uint16_t pixels) {
  if (!usablePin(clockPin) || !dataPins || !lanes || lanes > Capacity || !pixels)
    return false;
  for (uint8_t lane = 0; lane < lanes; ++lane) {
    if (!usablePin(dataPins[lane]) || dataPins[lane] == clockPin) return false;
    for (uint8_t previous = 0; previous < lane; ++previous)
      if (dataPins[lane] == dataPins[previous]) return false;
  }
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
  if (!mutex_) return false;
  // Allocate before replacing a working buffer. GPIO staging stays internal.
  auto* next = static_cast<uint8_t*>(heap_caps_calloc(
      size_t(lanes) * pixels, 3, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!next) return false;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  ready_ = false;
  free(rgb_);
  rgb_ = next;
  pixels_ = pixels;
  lanes_ = lanes;
  allLow_ = allHigh_ = 0;
  clockLow_ = clockPin < 32 ? 1UL << clockPin : 0;
  clockHigh_ = clockPin >= 32 ? 1UL << (clockPin - 32) : 0;
  digitalWrite(clockPin, LOW);
  pinMode(clockPin, OUTPUT);
  for (uint8_t lane = 0; lane < lanes; ++lane) {
    int pin = dataPins[lane];
    dataLow_[lane] = pin < 32 ? 1UL << pin : 0;
    dataHigh_[lane] = pin >= 32 ? 1UL << (pin - 32) : 0;
    allLow_ |= dataLow_[lane];
    allHigh_ |= dataHigh_[lane];
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  ready_ = true;
  xSemaphoreGive(mutex_);
  show(); // Latch an actual black frame at startup.
  return true;
}

void SharedClockOutput::setPixel(uint8_t lane, uint16_t pixel,
                                uint8_t r, uint8_t g, uint8_t b) {
  if (!mutex_) return;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  if (ready_ && lane < lanes_ && pixel < pixels_) {
    uint8_t* p = rgb_ + (size_t(lane) * pixels_ + pixel) * 3u;
    p[0] = r; p[1] = g; p[2] = b;
  }
  xSemaphoreGive(mutex_);
}

void SharedClockOutput::clear(uint8_t lane) {
  if (!mutex_) return;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  if (ready_ && lane < lanes_)
    memset(rgb_ + size_t(lane) * pixels_ * 3u, 0, size_t(pixels_) * 3u);
  xSemaphoreGive(mutex_);
}

void SharedClockOutput::setBrightness(uint8_t brightness) {
  if (!mutex_) { brightness_ = brightness; return; }
  xSemaphoreTake(mutex_, portMAX_DELAY);
  brightness_ = brightness;
  xSemaphoreGive(mutex_);
}

void SharedClockOutput::show() {
  if (!mutex_) return;
  xSemaphoreTake(mutex_, portMAX_DELAY);
  if (!ready_) { xSemaphoreGive(mutex_); return; }
  const uint32_t start = micros();
  const size_t bytes = SharedClockProtocol::frameBytes(pixels_);
  uint8_t values[Capacity];
  for (size_t byte = 0; byte < bytes; ++byte) {
    for (uint8_t lane = 0; lane < lanes_; ++lane)
      values[lane] = SharedClockProtocol::wireByte(
          rgb_ + size_t(lane) * pixels_ * 3u, pixels_, byte, brightness_);
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
      uint32_t low = 0, high = 0;
      for (uint8_t lane = 0; lane < lanes_; ++lane) {
        if (values[lane] & bit) { low |= dataLow_[lane]; high |= dataHigh_[lane]; }
      }
      // Change every data line while the common clock is LOW. Only touch our
      // own bits, including GPIOs in both register banks. Keep interrupts on
      // so the single Hall index pulse can still be timestamped.
      GPIO.out_w1tc = allLow_;
      GPIO.out1_w1tc.val = allHigh_;
      GPIO.out_w1ts = low;
      GPIO.out1_w1ts.val = high;
      GPIO.out_w1ts = clockLow_;
      GPIO.out1_w1ts.val = clockHigh_;
      asm volatile("nop; nop; nop; nop; nop; nop; nop; nop;");
      GPIO.out_w1tc = clockLow_;
      GPIO.out1_w1tc.val = clockHigh_;
    }
  }
  GPIO.out_w1tc = allLow_;
  GPIO.out1_w1tc.val = allHigh_;
  lastTransmitUs_ = micros() - start;
  xSemaphoreGive(mutex_);
}
