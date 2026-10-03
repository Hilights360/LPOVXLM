# Why the arms can't paint fast enough — and what actually fixes it

Analysis of the current `LPOVXLM.ino` output path, and what hardware (if any) the
bigger multi-arm units need.

**Headline: the ESP32-S3 is not the limiting part. `Adafruit_DotStar::show()` is.**
Two software changes get you 4× the angular resolution and effectively unlimited
arm count on the board you already have. A different MCU only becomes the answer
above ~16 arms or ~430 slices.

---

## What the code does today

Verified by reading the sketch, not assumed:

| | |
|---|---|
| MCU | ESP32-S3 (Arduino), display task pinned to Core 1, frame loading on Core 0 |
| LEDs | SK9822 / APA102 — clock + data, 32 bits per pixel |
| Output | `Adafruit_DotStar`, **2 SPI lanes**, GPIO 47/45 and 35/36 |
| Topology | 2 lanes × **2 arms chained back-to-back** = 4 arms (`MAX_ARMS = 4`) |
| Clock | `g_spiClockHz = 40 MHz` |
| Geometry | 144 px/arm default, 40 spokes, `g_fps = 40` |
| Source | FSEQ v2 from SD, sparse + zlib per frame. No E1.31/DDP. |

The critical path is `lanesCommit()`:

```cpp
for (uint8_t l=0; l<NUM_LANES; ++l) if (g_lanes[l]) g_lanes[l]->show();
```

**Two blocking calls, one after the other, no DMA.** Adafruit DotStar's `show()`
busy-transmits and returns when the last bit is out. So Core 1 stalls for the
full transmission of lane 0, *then* the full transmission of lane 1 — while the
SPI peripherals sit idle half the time each.

Using the sketch's own bit formula from `handleDiagSpi()`
(`32 + n*32 + 32`), at 40 MHz and 144 px/arm:

- one arm, alone on a lane — **116.8 µs**
- two arms chained on a lane — **232.0 µs**
- today's commit, two lanes sequentially — **464.0 µs**

---

## What that costs in angular resolution

Angular slices per revolution is the horizontal resolution of the image. Flicker-
free POV wants roughly 15–20 rev/s, so the 900–1200 rpm columns are the ones that
matter.

| configuration | arms | commit | 600 rpm | 900 rpm | **1200 rpm** | 1800 rpm |
|---|---|---|---|---|---|---|
| **TODAY** 2 lanes × 2 chained, blocking, sequential | 4 | 464 µs | 215 | 143 | **107** | 71 |
| same wiring, DMA, both lanes concurrent | 4 | 232 µs | 431 | 287 | **215** | 143 |
| 4 independent lanes concurrent | 4 | 117 µs | 856 | 570 | **428** | 285 |
| LCD_CAM 8-bit parallel, shared clock, DMA | **8** | 117 µs | 856 | 570 | **428** | 285 |
| LCD_CAM 16-bit parallel, shared clock, DMA | **16** | 117 µs | 856 | 570 | **428** | 285 |
| RP2350 PIO, 12 independent SMs + DMA | 12 | 117 µs | 856 | 570 | **428** | 285 |

Today's ceiling at 1200 rpm is **107 slices**. You are running 40 spokes, which
fits comfortably — but 40 spokes is 9° per column, and that coarseness is the
thing you are feeling as "can't paint fast enough." It is not a CPU shortage.

Note the bottom four rows are identical. Once transmission is concurrent and DMA-
driven, **arm count stops costing anything** and the limit becomes one arm's
transmission time. That is the whole game.

---

## Confirm it before acting — you already built the instrument

`/diag/spi` reports `transmitPercent`: transmission time as a fraction of the
spoke period. Hit it at your real working RPM and spoke count.

- **High (>50%)** — output-bound. Everything above applies, in order.
- **Low (<20%)** — the bottleneck is upstream: SD read, zlib inflate, or frame
  mapping. At 40 spokes × 144 px × 3 ch = 17,280 B per frame and 40 fps, that is
  ~691 KB/s of inflate output, which is real work for one core. Different problem,
  different fix (bigger sparse ranges, pre-decompressed cache in PSRAM, or
  dropping zlib for raw at the cost of card space).

Do not skip this. The table above is a model of the output path; the endpoint
measures the actual machine.

---

## The fixes, cheapest first

### 1. DMA, and let both lanes run at once — free, no hardware

Replace `Adafruit_DotStar` with ESP-IDF `spi_master` on SPI2 and SPI3, DMA-fed,
queued rather than blocking. Both lanes then transmit **simultaneously** and Core
1 gets the whole transmission window back instead of spinning in it.

**464 µs → 232 µs. Double the slices, plus a core's worth of headroom.** Do this
regardless of what hardware you eventually buy — every later option assumes it.

### 2. Stop chaining arms — but this is where the S3 runs out

One arm per lane is 117 µs instead of 232 µs, because a chained pair puts twice
the bytes on one wire. Four independent lanes would give 428 slices at 1200 rpm.

**The ESP32-S3 only has two general-purpose SPI peripherals** (SPI2/FSPI and
SPI3/HSPI). SPI0/SPI1 are wired to flash and PSRAM and are not yours. So four
independent hardware SPI lanes is not available, and this is the real wall — the
one that looks like "the ESP32 isn't fast enough" when it is actually "the ESP32
has two SPI ports."

### 3. LCD_CAM parallel — 16 arms for the price of one, still no new hardware

This is the one to build, and your unused `OUT_PARALLEL` enum was already
reaching for it.

SK9822 is **clock + data**. Every arm can share one clock line. The ESP32-S3's
LCD_CAM peripheral in i80 mode clocks out 8 or 16 bits per PCLK, DMA-fed. Give
each arm one data pin and a common clock, and **all 16 arms transmit in the time
of a single arm — 117 µs — no matter how many there are.**

The cost is bit-transposition: for each bit-time you need one word holding that
bit for every arm, so the frame has to be transposed from per-arm byte streams
into per-clock lane words. That is a fixed per-frame CPU cost on Core 0, and the
standard 32×32 bit-transpose trick makes it cheap. It is real work but it is work
you do once per frame, not once per pixel.

This is where the bigger units should land: **arm count becomes free.**

### 4. RP2350 (Pico 2) — only if you need more than 16 arms

3 PIO blocks × 4 state machines = **12 independent SMs**, each driving its own
APA102 lane with its own DMA channel, no transposition needed because each arm
gets its own natural byte stream. Dual Cortex-M33 at 150 MHz.

It is the textbook right answer for "many independent clocked serial lanes" — and
it would cost you the 97 KB of working sketch: the FSEQ v2 player, the web UI,
OTA, SD 4-bit, WiFi, the RPM/hall sync. Pico 2 W has WiFi but nothing like the
ESP32's web and OTA maturity.

**If you go this way, do it as a hybrid:** keep the ESP32-S3 as the brain doing
WiFi, SD, FSEQ decode and the web UI, and add a ~$1 RP2040/RP2350 as a dumb pixel
pusher fed over a fast link. You keep everything that works and buy only the
fan-out.

---

## Three gotchas found while reading

**GPIO 35/36 are lane pins — check what your module's PSRAM claims.** On ESP32-S3
modules with **octal** PSRAM (N8R8, N16R8) GPIO 35/36/37 are consumed by the PSRAM
interface. Since lane 1 currently uses 35 and 36 and evidently works, your module
must be quad-PSRAM, no-PSRAM, or PSRAM-disabled. Worth knowing for two reasons:
LCD_CAM 16-bit needs ~17 free GPIOs, and caching decompressed frames wants PSRAM.
You may not be able to have both.

**Don't raise the SPI clock. You are already past spec.** APA102 datasheets sit
around 20 MHz; SK9822 tolerates more because it has a proper latch, but 40 MHz
over slip rings and long arm wiring is aggressive already. The symptom of pushing
further is sparkle and colour glitches that look like software bugs. Parallelism
is the path, not clock rate.

**`MAX_ARMS = 4` is a software cap** in `ConfigTypes.h`, and `QuadMap.h` computes
`stride = spokes / arms` — so spoke count must stay divisible by arm count. Six
or eight arms means 48/240/360 spokes, not 40.

---

## Recommendation

1. Read `/diag/spi` at working RPM. Confirm output-bound before touching anything.
2. Do the DMA + concurrent-lane rewrite. Free, doubles resolution, no hardware.
3. Build LCD_CAM parallel for the bigger units. 16 arms, 428 slices at 1200 rpm,
   still the board you already have.
4. Only look at RP2350 if you need more than 16 arms — and then as a co-processor,
   not a replacement.

**Buy nothing yet.** The board is not the problem.
