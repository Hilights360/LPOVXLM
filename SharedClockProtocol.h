#pragma once
#include <stddef.h>
#include <stdint.h>
#include <array>
#include <string.h>

// One independent SK9822/APA102 strip per data pin. All strips share the clock.
// Include the SK9822 reset word and enough trailing clocks for the entire strip.
namespace SharedClockProtocol {
enum class Protocol : uint8_t { Sk9822 = 0, Apa102 = 1 };
constexpr size_t frameBytes(uint16_t pixels, Protocol protocol = Protocol::Sk9822) {
  const size_t tail = (pixels + 15u) / 16u;
  return 4u + size_t(pixels) * 4u + (protocol == Protocol::Sk9822 ? 4u + tail : (tail < 4u ? 4u : tail));
}

constexpr uint8_t wireByte(const uint8_t* rgb, uint16_t pixels, size_t byte,
                           uint8_t brightness, bool reverse = false, Protocol protocol = Protocol::Sk9822) {
  if (byte < 4u) return 0;
  const size_t payload = size_t(pixels) * 4u;
  byte -= 4u;
  if (byte >= payload) return protocol == Protocol::Sk9822 && byte < payload + 4u ? 0 : 0xFF;
  const size_t component = byte % 4u;
  if (!component) return 0xFF; // Full global brightness; scale RGB instead.
  if (!rgb) return 0;
  const size_t pixel = reverse ? pixels - 1u - byte / 4u : byte / 4u;
  // RGB in RAM, BGR on the wire.
  const uint8_t value = rgb[pixel * 3u + (3u - component)];
  return uint16_t(value) * (uint16_t(brightness) + 1u) >> 8;
}

// One byte per clock: bits 0..3 are the independent strip data levels.
// The other four LCD bus lanes stay low. RGB is stored as four planar strips.
constexpr std::array<uint64_t, 256> makeLaneTable() {
  std::array<uint64_t, 256> table{};
  for (unsigned value = 0; value < 256; ++value)
    for (unsigned bit = 0; bit < 8; ++bit)
      if (value & (0x80u >> bit)) table[value] |= uint64_t(1) << (8 * bit);
  return table;
}
inline constexpr auto LaneTable = makeLaneTable();
inline bool packBlackFourLanes(uint16_t pixels, uint8_t* destination, size_t capacity,
                               Protocol protocol = Protocol::Sk9822) {
  if (!pixels || !destination || capacity < frameBytes(pixels, protocol) * 8u) return false;
  for (size_t byte = 0; byte < frameBytes(pixels, protocol); ++byte) {
    const uint64_t packed = LaneTable[wireByte(nullptr, pixels, byte, 0, false, protocol)] * 15;
    memcpy(destination + byte * 8, &packed, sizeof(packed));
  }
  return true;
}
inline bool packFourLanes(const uint8_t* rgb, uint16_t pixels, uint8_t brightness,
                          uint8_t* destination, size_t capacity, Protocol protocol = Protocol::Sk9822) {
  if (!rgb || !pixels || !destination || capacity < frameBytes(pixels, protocol) * 8u) return false;
  // Headers and tails are shared by all lanes. Pack only RGB in the hot loop,
  // with constant lane shifts instead of generic 64-bit variable shifts.
  memset(destination, 0, 32);
  size_t out = 32;
  const size_t stride = size_t(pixels) * 3;
  const unsigned scale = unsigned(brightness) + 1;
  for (size_t pixel = 0; pixel < pixels; ++pixel) {
    memset(destination + out, 0x0F, 8); // Full global brightness on all lanes.
    out += 8;
    for (int component = 2; component >= 0; --component) {
      const size_t index = pixel * 3 + component;
      const uint64_t packed = LaneTable[(unsigned(rgb[index]) * scale) >> 8]
        | (LaneTable[(unsigned(rgb[stride + index]) * scale) >> 8] << 1)
        | (LaneTable[(unsigned(rgb[2 * stride + index]) * scale) >> 8] << 2)
        | (LaneTable[(unsigned(rgb[3 * stride + index]) * scale) >> 8] << 3);
      memcpy(destination + out, &packed, sizeof(packed));
      out += sizeof(packed);
    }
  }
  if (protocol == Protocol::Sk9822) { memset(destination + out, 0, 32); out += 32; }
  memset(destination + out, 0x0F, frameBytes(pixels, protocol) * 8u - out);
  return true;
}
}
