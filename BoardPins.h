#pragma once
#include <stdint.h>

// Rebuilt 16-connector PCB. Data map traced from the supplied layer images;
// GPIO42 LED clock and GPIO1 motor speed PWM confirmed by the board designer.
// GPIO numbers, NOT header positions. Connector IDs identify the data nets;
// rotor arm angles are specified separately below. Connector 16-13 is arm 4;
// the duplicate four-arm label in the reference image has been corrected.
namespace BoardPins {
constexpr uint8_t OutputCount = 16;
constexpr int OutputData[OutputCount] = {
  21, 47, 48, 45, 38, 39, 40, 41, 18, 17, 16, 15, 7, 6, 5, 4
};
constexpr int Clock = 42;
constexpr int MotorSpeedPwm = 1; // Assigned motor speed PWM; never driven by LED code.
constexpr uint8_t ActiveConnectors[] = {1, 5, 9, 13};
constexpr int ArmData[] = {OutputData[0], OutputData[4], OutputData[8], OutputData[12]};
// Connector identity is fixed. Arm order and rotation direction are saved
// through LED wiring setup; neither changes these GPIO assignments.
// The user confirmed that the first LED / DI-CI input is at the hub.
constexpr bool ArmInputAtHub[] = {true, true, true, true};
// Default images start at the hub. Older exports may start at the tip; that
// is a per-sequence property, not a property of the PCB's input connector.
constexpr bool ArmReverse[] = {!ArmInputAtHub[0], !ArmInputAtHub[1], !ArmInputAtHub[2], !ArmInputAtHub[3]};
constexpr unsigned imagePixelToWire(unsigned arm, unsigned pixel, unsigned pixels, bool tipFirst = false) {
  return (ArmReverse[arm] != tipFirst) && pixel < pixels ? pixels - 1 - pixel : pixel;
}
constexpr int Hall = 3;
constexpr int Encoder = 8; // Reserved; single Hall index is the only position input.
// GPIO48 now feeds connector 16-3; the onboard RGB indicator cannot be driven.
constexpr int StatusPixel = -1;
// SD net labels confirmed in the PCB close-up. GPIO14 is DAT2, not detect.
constexpr int SdClk = 11;
constexpr int SdCmd = 12;
constexpr int SdD0 = 10;
constexpr int SdD1 = 9;
constexpr int SdD2 = 14;
constexpr int SdD3 = 13;
constexpr int SdCd = -1; // No dedicated detect signal; probe the card over SDMMC.
constexpr uint8_t InitialArms = 4;
constexpr uint8_t PulsesPerRevolution = 1;

constexpr bool usableOutputPin(int pin) {
  return pin >= 0 && pin <= 48 && !(pin >= 22 && pin <= 37);
}
constexpr bool outputPinsValid() {
  const int reserved[] = {MotorSpeedPwm, Hall, Encoder, StatusPixel, SdClk, SdCmd, SdD0,
                         SdD1, SdD2, SdD3, SdCd, 0, 2, 19, 20, 43, 44};
  if (!usableOutputPin(Clock)) return false;
  for (int r : reserved) if (Clock == r) return false;
  for (uint8_t i = 0; i < OutputCount; ++i) {
    int p = OutputData[i];
    if (!usableOutputPin(p) || p == Clock) return false;
    for (int r : reserved) if (p == r) return false;
    for (uint8_t j = 0; j < i; ++j) if (p == OutputData[j]) return false;
  }
  return true;
}
static_assert(outputPinsValid(), "PCB outputs overlap another peripheral or memory");
static_assert(InitialArms == sizeof(ArmData) / sizeof(ArmData[0]), "Four-arm map mismatch");
static_assert(InitialArms == sizeof(ArmInputAtHub) / sizeof(ArmInputAtHub[0]), "Four-arm input location mismatch");
static_assert(InitialArms == sizeof(ArmReverse) / sizeof(ArmReverse[0]), "Four-arm image direction mismatch");
}
