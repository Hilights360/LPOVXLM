#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// GPIO driver for initial board bring-up. This is synchronous software output,
// not LCD_CAM/DMA, and has no fixed or guaranteed MHz rate.
class SharedClockOutput {
 public:
  static constexpr uint8_t Capacity = 16;
  bool begin(int clockPin, const int* dataPins, uint8_t lanes, uint16_t pixels);
  void setPixel(uint8_t lane, uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);
  void clear(uint8_t lane);
  void setBrightness(uint8_t brightness);
  void show();
  bool ready() const { return ready_; }
  uint32_t lastTransmitUs() const { return lastTransmitUs_; }

 private:
  SemaphoreHandle_t mutex_ = nullptr;
  uint8_t* rgb_ = nullptr;
  uint16_t pixels_ = 0;
  uint8_t lanes_ = 0;
  uint8_t brightness_ = 255;
  uint32_t dataLow_[Capacity] = {};
  uint32_t dataHigh_[Capacity] = {};
  uint32_t allLow_ = 0, allHigh_ = 0;
  uint32_t clockLow_ = 0, clockHigh_ = 0;
  bool ready_ = false;
  uint32_t lastTransmitUs_ = 0;
};
