#pragma once
#include <math.h>

// Labeling mode: xLights “lower wedge at boundary” (matches what you see: 1,11,20,30)
// or “round to center” (mathematically exact: 1,11,21,31)
enum SpokeLabelMode { ROUND_TO_CENTER, FLOOR_TO_BOUNDARY };

// Convert 1-based xLights spoke to 0-based internal index
static inline int spoke1BasedToIdx0(int spoke1based, int spokes=40) {
  int s = (spoke1based - 1) % spokes;
  return (s < 0) ? (s + spokes) : s;
}

// Angle (deg) -> 0-based spoke index, with selectable labeling
static inline int angleToSpokeIdx0(float deg, int spokes, SpokeLabelMode mode) {
  float idx = fmodf((deg / 360.0f) * (float)spokes, (float)spokes);
  if (idx < 0) idx += spokes;
  if (mode == ROUND_TO_CENTER) return ((int)floorf(idx + 0.5f)) % spokes;
  return (int)floorf(idx); // FLOOR_TO_BOUNDARY → matches xLights’ “1,11,20,30”
}

// 0-based spoke for arm k given startSpokeIdx0, total spokes, and arms
static inline int armSpokeIdx0(int armK, int startSpokeIdx0, int spokes, int arms) {
  const int stride = spokes / arms;     // e.g., 40/4 = 10
  return (startSpokeIdx0 + armK * stride) % spokes;
}

// Channel math: how many channels per spoke (you pass this in)
static inline size_t armChannelOffset(int armK, int spokes, int arms, size_t channelsPerSpoke) {
  const int stride = spokes / arms;     // e.g., 10
  return (size_t)armK * (size_t)stride * channelsPerSpoke;
}
