# 8 or 16 arms: main highlights

**Keep the ESP32-S3 as the planned controller. Design for sixteen intact
144-pixel clocked strips, with eight populated initially.** Use 256 angular
positions and LCD_CAM parallel DMA: one data signal per arm, plus a shared
clock buffered separately to each arm. The peripheral supports both 8-bit and
16-bit buses; this application still needs a working driver and bench validation.
[Espressif I80 documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/lcd/i80_lcd.html)

## What changes with sixteen arms?

| Item | 8 arms | 16 arms |
|---|---:|---:|
| Total pixels | 1,152 | 2,304 |
| Parallel data GPIOs | 8 | 16 |
| SN74AHCT245 arm buffers | 2 | 4 |
| SN74LVC125A clock fan-out buffers | 1 | 1 |
| Passes per image position at 400 rpm | 53.3/sec | 106.7/sec |
| LED current at 60 mA/pixel | 69.1 A | 138.2 A |
| LED power at 5 V, same assumption | 346 W | 691 W |

Reserve all sixteen outputs now; adding the second eight should require
populating the extra buffers/connectors and changing configuration, with no
rewiring of the first eight. All arms have independent data; none are chained
to another arm.

## Timing: unchanged when all arms transmit together

For APA102, each 144-pixel transmission needs
**32 + 144 × 32 + 72 = 4,712 clock cycles**, including end clocks.
At 20 MHz: **235.6 µs color + 235.6 µs black = 471.2 µs**.
[Protocol measurements](https://cpldcpu.com/2014/11/30/understanding-the-apa102-superled/)

| Speed / clock | Time per position | Color + black | Remaining |
|---|---:|---:|---:|
| 400 rpm boundary / 20 MHz | 585.9 µs | 471.2 µs | **114.7 µs** |
| 350 rpm / 20 MHz | 669.6 µs | 471.2 µs | **198.4 µs** |
| 300 rpm / 16 MHz | 781.3 µs | 589.0 µs | **192.3 µs** |

These figures exclude software overhead. Sixteen arms double DMA traffic and
pixel preparation, so matching the wire budget alone does not prove performance.
**16 MHz does not fit 400 rpm** with full color and black frames.

A useful sixteen-arm starting point is **300 rpm, 16 MHz, 50% duty**:
80 passes per image position per second. Confirm the actual strip's clock
limit, PWM and latch behavior; SK9822 needs its own reset/latch sequence.

## Pins and hardware

- Use the **YD-ESP32-S3 with N16R8 octal PSRAM**, subject to checking its marking.
  **GPIO35/36/37 are reserved for memory; GPIO38 is available on this YD board.**
  Its onboard RGB LED uses GPIO48. These are GPIO numbers, not header positions.
  [YD manufacturer pinout](https://github.com/vcc-gnd/YD-ESP32-S3/blob/main/README.md)
- Preserve four-bit SD, Hall GPIO5, USB and serial. Sixteen outputs need
  **GPIO45 as data with a boot pull-down**, plus **GPIO3 reserved for LCD D/C
  with a pull-up**. This replaces the earlier blanket avoidance of those pins.
  See the complete [GPIO and buffer plan](PCB_ROUTING_PLAN.md) before routing.
- Add **four 5 V SN74AHCT245s** for sixteen arms, each handling four data/clock
  pairs, and **one 3.3 V SN74LVC125A** for four clock branches. Populate two
  AHCT245s for eight arms. Include bypass capacitors and output resistor footprints.
  [AHCT245](https://www.ti.com/lit/ds/symlink/sn74ahct245.pdf),
  [LVC125A](https://www.ti.com/lit/ds/symlink/sn74lvc125a.pdf)
- Use separate fused arm power distribution. Size copper, wiring and connectors
  for the selected current limit; do not send 138 A through the shown carrier
  tracks. At unrestricted full white, provision roughly **800–1,000 W at 5 V**
  for sixteen arms, versus 400–500 W for eight. Actual strip current must be checked.
  [Example strip specification](https://www.adafruit.com/product/2241)

## Blanking and implementation

**Write black after every color update. Buffer /OE does not blank stored LED
colors.** Start with 50% duty; APA102 updates progressively along the arm.
Check actual light pulses at both ends, not just electrical clock timing.

The existing firmware uses sequential software SPI, caps arms at four, and has
a stub parallel path. Future work needs selectable 8/16-arm DMA, phase mapping,
and buffering that keeps SD/Wi-Fi work from delaying black frames.
For equally spaced arms, offsets are **32 columns for eight, 16 for sixteen**.

**Comparison with the earlier assessment:** the eight-arm timing remains valid;
sixteen parallel arms increase refresh, power and memory traffic without
lengthening each transmission. Two daisy-chained outputs still fail the
400 rpm / 256-position color-plus-black budget.

*Documentation only; no firmware or PCB source changes.*
