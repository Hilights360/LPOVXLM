# Rebuilt PCB: initial four-arm firmware

The active firmware is now the native ESP-IDF application in `main/`. See
[`ESP_IDF_PORT.md`](ESP_IDF_PORT.md) for the port and current validation.

This supersedes the **proposed** GPIO table in `PCB_ROUTING_PLAN.md` for this
PCB. The output mapping below is inferred by tracing the supplied front, back,
and combined-layer screenshots through the buffers. The SD mapping was corrected
from the net labels at the ESP header in the later SD connector close-up.
The LED clock and motor speed PWM pins were later explicitly confirmed by the
designer as GPIO42 and GPIO1. The remaining data mapping has not been verified
against a CAD netlist or continuity measurements. References are saved in
[`hardware/reference`](hardware/reference).

The next revision must follow the
[in-house assembly requirements](hardware/DESIGN_REQUIREMENTS.md): no 0402
parts, with 0805 as the working default for resistors and ceramic capacitors.

## Initial connections

| Logical arm | Physical angle relative to arm 1 | PCB connector | DI GPIO | Common CI GPIO |
|---|---|---|---:|---:|
| 1 | 0 degrees | 1 / 16-1 | 21 | 42 |
| 2 | -90 degrees | 4-2 / 16-5 | 38 | 42 |
| 3 | -180 degrees | 4-3 / 16-9 | 18 | 42 |
| 4 | -270 degrees | 16-13 | 7 | 42 |

The user verified clockwise rotation viewed from the ESP32 component side,
with counterclockwise arm numbering. The stationary Connector colors order
clockwise from red is **red, white, blue, green**. This supersedes the earlier
physical-position inference from PCB screenshots. GPIO assignments are retained;
`BoardPins::ArmAngleDegrees` records rotor geometry separately from connector IDs.

The duplicate **4-3** label in the reference image has been corrected on the
PCB, as confirmed by the designer. **16-13 is logical arm 4**. These are four
independent strips at quarter-turn intervals. There is no second chained strip
or automatic alternating reversal.
The user confirmed that strip inputs are at the hub. The default sequence
mapping sends source pixel zero to the hub. Tip-first exports use the saved
per-sequence radial reversal; input location alone cannot establish the
sequence's pixel order. `ArmInputAtHub` anchors brightness shaping to the
physical hub after that mapping.

| Other connection | GPIO |
|---|---:|
| Motor speed PWM, assigned; excluded from LED output | 1 |
| Hall sensor, active LOW | 3 |
| Encoder header, reserved and unused | 8 |
| SD CLK / CMD / D0 | 11 / 12 / 10 |
| SD D1 / D2 / D3 | 9 / 14 / 13 |
| SD card detect | None configured (`SdCd = -1`); probe over SDMMC |
| USB D- / D+ | 19 / 20, reserved |
| UART TX / RX | 43 / 44, reserved |

Hall configuration starts at **one falling-edge pulse per revolution**. The
first boot of this PCB revision migrates old saved arm count, pulse count,
edge selection, and chained-arm channel mapping. Brightness, pixels per arm,
Wi-Fi, and playback preferences remain. A legacy SD backup cannot re-enable
the incompatible two-lane SPI mode.

## Checks needed on the PCB

- **The September 18 SD close-up has the correct CLK/VDD order.** Reading
  the socket pad row from DAT1 toward DAT2, its net labels are DAT1, DAT0,
  GND, **CLK, 3.3V**, CMD, DAT3, DAT2. The
  [GCT MEM2067 drawing, page 1](https://gct.co/files/drawings/mem2067.pdf)
  numbers those pads 8 through 1 and assigns **P5=CLK, P4=VDD**. The expected
  order there is therefore **CLK, 3.3V**, matching the latest image. The earlier
  suspected swap is withdrawn for this image. The designer also confirms five
  external 10k pull-ups, on CMD and DAT0-3. **CLK / GPIO11 has no external
  pull-up**, explicitly reconfirmed by the designer on September 21. The
  designer subsequently reported that continuity checks passed. These establish the intended wiring, but do not
  exclude intermittent contacts or signal/power problems under load. The live
  [SD investigation](SD_TROUBLESHOOTING.md) reproduced data CRC failures during
  4-bit transfers; socket supply and signal waveforms are the next checks.
- GPIO48 is routed as an optional strip data output. The old onboard RGB Hall
  status indication has therefore been removed; its NeoPixel pulses would
  interfere with that output. Hall status remains available in the web UI.
- GPIO3 (Hall) and GPIO45 (optional output) are boot strapping pins. Check their
  reset levels with the fitted hardware. The earlier SD D0 assignment to GPIO46
  and the associated SD pull-up warning were incorrect; SD D0 is GPIO10.

The earlier GPIO14 card-detect assignment was also incorrect: it is DAT2.
Reading its pull-up as "no card" could prevent any mount attempt. The firmware
now probes the card using SDMMC without a dedicated card-detect input.

These are findings from the images, not electrical verification or a DRC pass.
The [Espressif SD pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sd_pullup_requirements.html)
describe the SD connections.

Correction: the earlier GPIO42 grounding warning is withdrawn. It came from
an unverified interpretation of the upper-right pad of the bottom-center
buffer, not an identified GND net label or verified ground connection.
The designer has confirmed **GPIO42 is the common LED clock** and **GPIO1
is motor speed PWM**. The former GPIO1 clock assignment was wrong. Firmware now
uses GPIO42 for normal output, DMA, and signal checks. GPIO1 is assigned to
motor speed PWM and excluded from LED output. Motor PWM control has not yet
been implemented.

## Optional connectors

The full screenshot-derived data map is recorded centrally for later expansion:

| Connector | DI GPIO | Connector | DI GPIO |
|---|---:|---|---:|
| 16-1 | 21 | 16-9 | 18 |
| 16-2 | 47 | 16-10 | 17 |
| 16-3 | 48 | 16-11 | 16 |
| 16-4 | 45 | 16-12 | 15 |
| 16-5 | 38 | 16-13 | 7 |
| 16-6 | 39 | 16-14 | 6 |
| 16-7 | 40 | 16-15 | 5 |
| 16-8 | 41 | 16-16 | 4 |

Unused data lines are held LOW. `MAX_ARMS` stays at four; selecting fewer arms
disables the end of this four-position list and does not choose a different set
of connectors. Expanding to eight or sixteen requires a new active-connector
map, geometry/settings updates, and performance testing.

## Build and bench validation

Run `./build.ps1` in PowerShell. The script uses **ESP-IDF 5.3.3**, with
**16 MB QIO flash, OPI PSRAM, the existing custom partition table, playback
on Core 0, display on Core 1**. It builds into `build/idf` and does not flash
the board or overwrite the previous exported Arduino firmware.

GPIO35/36/37 are no longer configured as outputs. These are memory signals on
the N16R8 octal-PSRAM module; see
[Espressif's GPIO restrictions](https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32s3/api-reference/peripherals/gpio.html).
FSEQ and decompression buffers prefer PSRAM, with internal-RAM fallback.
Startup logs and `/diag/spi` report detected PSRAM; verify the expected 8 MB
on the fitted N16R8 module.

The driver sends four distinct bitstreams together on the common clock,
including actual black frames and length-dependent end clocks. **LED setup**
on the **LEDs** page (`/leds#setup`) selects SK9822 framing with its extra reset
word, or APA102 framing.
Both use BGR wire order. Normal playback uses synchronous GPIO. A separate
[stationary LCD_CAM/GDMA test](DMA_TEST.md) now compares that driver with
DMA at 4, 8, or 16 MHz, then restores GPIO. Neither mode establishes an
operating-RPM guarantee without measurements on the PCB.
`/diag/spi` now reports measured transmit time instead of the old, ineffective
SPI-frequency estimate. See the
[SK9822 protocol measurements](https://cpldcpu.github.io/2016/12/13/sk9822-a-clone-of-the-apa102/).

Start stationary at low brightness on **LEDs** (`/leds`). Solid colors,
all-arm fade, connector colors, and Arm RGB Test check output without an SD
card or Hall pulse. For dark strips, use the page's LOW/HIGH/1 Hz signal
check to trace GPIO42 (clock) and GPIO21/38/18/7 (data) from the ESP header
through the buffers to each DI/CI input. Firmware now explicitly configures
the GPIO pad function and enables input readback; this is not proof of the
reported no-light fault's cause or resolution. Verify SD mounting,
one Hall count per magnet pass, and PSRAM before testing rotation. No physical
board or oscilloscope validation has been performed here.

The native ESP-IDF application and bootloader build locally. Host format and
protocol checks are in `tests/native_format_test.cpp`; web browser checks are
in `tests/web_smoke.cjs`, `tests/wifi_setup.cjs`, and `tests/led_pages.cjs`.
These latest changes have not been flashed here. Current application
output is `build/idf/lpovxlm.bin`. The earlier Arduino build is retained in
`build/pcb16-four-arm` for reference.
