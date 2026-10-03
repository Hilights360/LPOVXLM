#include "SharedClockProtocol.h"
#include "BoardPins.h"
using namespace SharedClockProtocol;

constexpr uint8_t colors[] = {0x12, 0x34, 0x56, 0xAB, 0xCD, 0xEF};
static_assert(frameBytes(144) == 593, "144 pixels need 9 end bytes plus reset");
static_assert(frameBytes(1024) == 4168, "End clocks must scale with strip length");
static_assert(wireByte(colors, 2, 0, 255) == 0, "Start frame");
static_assert(wireByte(colors, 2, 3, 255) == 0, "All four start bytes");
static_assert(wireByte(colors, 2, 4, 255) == 0xFF, "Pixel brightness header");
static_assert(wireByte(colors, 2, 5, 255) == 0x56, "BGR blue");
static_assert(wireByte(colors, 2, 6, 255) == 0x34, "BGR green");
static_assert(wireByte(colors, 2, 7, 255) == 0x12, "BGR red");
static_assert(wireByte(colors, 2, 9, 255) == 0xEF, "Independent next pixel");
static_assert(wireByte(colors, 2, 5, 255, true) == 0xEF, "Reversed strip");
static_assert(wireByte(colors, 2, 9, 255, true) == 0x56, "Reversed final pixel");
static_assert(wireByte(colors, 2, 5, 0) == 0, "Zero brightness");
static_assert(wireByte(colors, 2, 5, 127) == 0x2B, "Half brightness");
static_assert(wireByte(nullptr, 2, 4, 255) == 0xFF, "Black retains valid header");
static_assert(wireByte(nullptr, 2, 5, 255) == 0, "Black color payload");
static_assert(wireByte(colors, 2, 12, 255) == 0, "SK9822 reset starts");
static_assert(wireByte(colors, 2, 15, 255) == 0, "SK9822 reset ends");
static_assert(wireByte(colors, 2, 16, 255) == 0xFF, "APA102 propagation clocks");
static_assert(BoardPins::outputPinsValid(), "Peripheral and memory pin conflicts");
static_assert(!BoardPins::usableOutputPin(35) && !BoardPins::usableOutputPin(36) &&
              !BoardPins::usableOutputPin(37), "Octal PSRAM pins must stay reserved");
static_assert(BoardPins::PulsesPerRevolution == 1, "One magnetic index per turn");
