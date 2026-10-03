# DMA test on the four-arm PCB

Normal sequence playback and rotation stability tests use parallel DMA at **20 MHz**
from Build 16, following the user's visual tests. They previously ran at 8 MHz.
The user confirmed 20 MHz displays correctly; 26.67 and 40 MHz do not.
The benchmark remains a separate comparison of GPIO and DMA. Stopping playback
or finishing a benchmark blanks the LEDs and restores GPIO for static diagnostics.

## Sustained strip clock test

Open **Speed test** in the header, or `/speed-test`. This checks whether the
physical strips display cleanly while receiving repeated DMA transfers at
**2, 4, 8, 10, 16, 20, 26.67, or 40 MHz**. These rates follow hardware divider
steps. They are test choices, not a claim about the fitted strips'
electrical rating. Start at 4 MHz with the rotor stopped and inspect all arms.

The current driver uses an 80 MHz clock divided by an integer. At the upper
end, divisors 4, 3, and 2 produce 20, approximately 26.67, and 40 MHz. Build 15
adds the previously omitted divide-by-three option. The integer-Hz request
must be **26,666,666**, rounded down; asking IDF 5.3.3 for 26,670,000 or
26,666,667 Hz selects divisor 2 and produces 40 MHz. The page sends integer Hz
and displays the intermediate result as 26.67 MHz. Arbitrary rates remain
rejected. User observation: 20 MHz displays correctly; 26.67 and 40 MHz do not.

Build 15 was installed with settings preserved. The device reported 6,885
transfers at the divide-by-three setting with no output error and about
201 microseconds mean submission-to-completion time. The run was cancelled
after 14.46 seconds, before its 15-second timeout, and returned to GPIO output.
This confirms the new request and cancellation path. The user subsequently
reported that 26.67 MHz displayed poorly and selected 20 MHz for playback.

Choose 15, 30, 60, or 120 seconds. Each run repeats a 12-second pattern: solid
red, green, blue, white, RGBW blocks with a moving white marker, and alternating
white/dark pixels. Each phase lasts two seconds. Brightness is uniform and
capped at 10% (or regular brightness if lower). Center dimming and angular
duty/strobe are bypassed for this stationary test; saved settings are unchanged.
Zero regular brightness prevents starting. No SD card or Hall pulse is needed.

**Looks correct** and **Shows errors** stop and record the user's visual
observation. Looks correct becomes available after a full 12-second pattern
cycle. The ESP cannot read back LED correctness; DMA completion only confirms
transmission. Results include the selected rate, protocol, arm/pixel counts,
brightness, duration, transfer timing, and measured update rate. Observations
stay in that browser for that controller address and can be downloaded as JSON.

Each transfer is followed by a short scheduling pause so controls remain
responsive. Reported update rate includes pattern construction and scheduling;
it is not a maximum playback FPS or rotor RPM. A device-side timeout ends the
run even if the browser disconnects. Stop/end/error blanks at 4 MHz before
restoring GPIO output. Playback and rotation patterns use **20 MHz**;
this page does not save a new playback clock or operate the motor.

API: `POST /diag/strip-speed` with URL-encoded `hz=26666666&seconds=30` starts
the intermediate-rate run; legacy `mhz=4&seconds=30` is also accepted.
`action=stop` cancels only this test. `GET /diag/strip-speed` reports its result.
Status mode ID is 11. Starting another light mode cancels the speed test.
Invalid rates/durations are rejected before interrupting the existing mode.

Build 14 was installed and checked on the device with four 144-pixel APA102
arms. A 15-second 4 MHz run completed 7,128 transfers without a reported output
error and stopped automatically. Cancellation, switching to Quarter colors,
invalid-parameter rejection, and the live page were checked. Settings were
unchanged and the quarter pattern was restored. This verifies the software
lifecycle; no visual pass has been recorded for any tested clock rate.

## Rotation stability patterns

Open **LEDs > LED tests > Rotation stability tests**:

- **Quarter colors** paints the first quarter of image spokes red, then green,
  blue, and white. With 80 spokes, these are 1-20, 21-40, 41-60, and 61-80.
- **Alternating spokes** paints red, green, blue, white, repeating one color
  per spoke around the disk. With 80 spokes, this gives 20 RGBW groups.

Every pixel along an arm gets the color for its angular position. All arms
use their physical connector offsets, saved global phase, and individual arm
phase, so colors stay tied to the single magnetic reference on GPIO3. These
tests use the same saved brightness, spoke count, duty/strobe, DMA driver, and
late-paint guards as playback. No SD file is opened or read. Changing between
tests stops the preceding pattern; **Stop lights** blanks and exits the test.
The LEDs remain dark without rotation/pulses, at zero brightness/duty, or when
the requested paint/blank windows cannot accommodate the LED transfers.

The verified rotor turns clockwise from the ESP32 component side, while arm
numbers run counterclockwise. Their base angles are **0, -90, -180, -270 degrees**.
The original positive offsets made successive arms alternate red/blue or
green/white in one quarter, despite matching spoke timing. The user confirmed
four steady quarters after reversing arm indexing. This geometry is shared by
both test patterns and normal playback; GPIO assignments are unchanged.

Build 10 makes the confirmed arm geometry permanent. It was installed over
OTA and checked at about 100 RPM with 80 spokes, 50% brightness, and 60% duty.
Quarter colors resumed using DMA with no output errors or timing-limit warning.
The temporary 180-degree corrections on arms 2 and 4 were removed; additional
arm phases are zero again so the geometry correction is applied only once.

Spoke counts not divisible by four split into quarters differing by at most
one spoke. Alternating colors restart with red at spoke 1 each revolution,
so a count divisible by four gives an uninterrupted RGBW repeat at the seam.
Counts below four cannot show all four quarter colors. The **Display duty (%)**
control under Rotation stability tests reads and saves the same duty setting
as Controller, and applies while a test is running. Use **Controller > Settings**
to change spoke count, angular strobe, or phase; the LEDs page displays the
active timing and explains when angular strobe sets the lit width.

API: `POST /led/spokes` with URL-encoded `pattern=quarters` or
`pattern=alternating`. Status mode IDs are 9 and 10 respectively. Invalid
patterns return an error without interrupting the current pattern.

Build 8 verification (September 18, 2026): native tests covered all 80 spoke
positions across four arms, phase wrap, repeated revolutions, and uneven spoke
counts. Browser tests covered both controls, Hall-wait messages, zero duty,
error handling, and mobile layout. OTA installation preserved device settings;
both modes enabled DMA with zero sequence frame reads, and Stop lights exited
DMA. The device had no magnetic pulses during that live check, so the actual
rotating color pattern still requires visual confirmation.

## Playback timing and SD recovery

Packing uses a byte lookup table, and black frames use a prebuilt DMA buffer.
SD frame reads and zlib decoding run outside the display lock; a complete frame
is published by swapping buffers. New animation frames are adopted on a spoke
boundary. A late or unachievable paint window is skipped and reported in the
web UI instead of starting a paint that cannot finish before blanking.

A playback frame load exceeding **100 ms** stops and blanks the display.
Three consecutive read/decode failures also stop playback. A cancelled read
cannot publish into a stopped or replacement sequence. SD remount/file changes
are deferred until that read releases the card; this avoids unmounting an active
driver. The existing optional 8-second task watchdog remains the last resort
if the reader never returns. The 100 ms timeout covers playback frame loading;
initial file opening and SD mounting still use the SD driver's own timeouts.

`/diag/timing` includes frame-load duration, SD stall count, skipped paints,
`timingLimited`, and `maxBlankStartLate_us` (delay before starting a due blank or
replacement transfer). `/diag/spi` identifies the active driver and transfer
times. Timing counters are software measurements, not an optical measurement
of individual LED turn-off times.

### Build 6 device verification (September 18, 2026)

- Installed over OTA; `/status` confirmed Ver1.2 Build 6.
- `4_Spokes_80_POV.fseq` played with 8 MHz DMA, 80 image spokes, four arms,
  144 pixels per arm, and 60% duty. Observed rotation was approximately
  81–118 RPM. The user confirmed consistent visible spoke widths.
- Typical color transfers were about 1.2 ms and cached black transfers about
  0.62 ms, compared with approximately 2.1 ms for the previous GPIO path.
  The lit/black playback mix averaged about 0.89 ms over one observed loop;
  this is not a guarantee for every frame or higher rotation speeds.
- A valid black frame followed by corrupt zlib data stopped playback after
  three failures, without restarting the controller.
- An all-black uncompressed fixture on a temporarily slowed 400 kHz SD bus
  triggered the 100 ms frame-load escape. Status remained reachable and
  reported Stopped with the timeout error. The SD bus was then restored to
  1-bit / 10 MHz, both temporary fixtures were removed, and the sequence
  resumed successfully. The resulting `sdStalls = 1` is from this deliberate
  test; it is a count since boot, not an active error.
- Host tests block the actual reader inside an injected `fread`, verify that
  close/cancel does not wait, reject publication of a stale frame, and prevent
  remount while the old read is active. They also cover failed reads and late
  paint windows. Existing protocol/packing and controller/LED browser tests
  pass. The firmware compiles and fits the OTA partition.

Verification snapshots and test fixtures are under `build/diagnostics/`.

Install the application binary, **`build/idf/lpovxlm.bin`**, using
**Firmware updates > Install over
Wi-Fi**, or run `./build.ps1 -Action flash -Port COM29` from PowerShell.
Reconnect to POV-Spinner (password `POV123456`), open http://192.168.4.1/,
and reload the page after the restart.

## Run the test

1. Keep the spinner stationary. Open **LEDs** (`/leds`). In **LED setup**, select
   the fitted LED protocol, four active arms, and the correct **Pixels per arm**.
   Neither a Hall pulse nor an SD sequence is needed.
2. On the same page, open **DMA speed test**.
   Select **4 MHz** and click **Run DMA test**.
3. The test sends 10 GPIO frames, followed by 100 DMA frames, in about six
   seconds. It alternates colors and black at up to 10% brightness. Arm 1 is
   red, arm 2 green, arm 3 blue, arm 4 white, with a moving white marker.
   Brightness zero keeps it dark. Each frame drives all four lanes together.
4. Read the results below the controls. **Complete** means all expected
   hardware completion interrupts arrived; confirm the LEDs also look right.
   Copy the results JSON when comparing runs. They remain at `/diag/dma`
   until another run or reboot, even after normal output statistics change.
5. Repeat at **8 MHz**, then **16 MHz** after checking the waveform and
   correctness. Speed alone is insufficient if pixels or colors are wrong.
   **Stop DMA test** cancels a run. Completion/cancellation blanks the strips
   and restores normal GPIO output. Configured background/autoplay can resume
   after its usual idle delay; the test does not permanently enable DMA.

## What is measured

| Result | Meaning |
| --- | --- |
| `gpio.meanTotal_us` | Mean GPIO call time, including wire-byte calculation and output |
| `dma.meanPack_us` | Mean time to pack RGB into the parallel DMA buffer |
| `dma.meanSubmitToDone_us` | Mean elapsed time from driver submission to the LCD transfer-complete ISR |
| `dma.meanTotal_us` | Mean complete call duration, including packing, submission, DMA and task wake-up |
| `dma.maxTotal_us` | Slowest measured DMA call; useful for spotting scheduling delays |
| `speedup` | GPIO mean total divided by DMA mean total |
| `theoreticalWire_us` | Calculated clocking time, **not a measurement** |

The benchmark discards the first DMA transfer to exclude initial device
configuration. The recorded samples alternate lit/black frames and include
all framing clocks for the selected protocol. Driver errors/timeouts produce **failed**, not a
successful sample. A stalled transfer disconnects the clock and retains DMA
memory until it finishes; if it remains stalled, the page requests a reboot.
An interrupted run reports **cancelled** with its partial samples.

For 144 pixels in **SK9822** mode, each strip needs 4,744 clocks per frame.
Four strips run in parallel, so do not multiply these times by four:

| Shared clock | Calculated wire time for one frame | Color + black frames |
| --- | ---: | ---: |
| 4 MHz | 1,186 us | 2,372 us |
| 8 MHz | 593 us | 1,186 us |
| 16 MHz | 296.5 us | 593 us |

**APA102** mode omits the extra SK9822 reset word: 4,712 clocks at 144 pixels,
or 1,178 / 589 / 294.5 us at 4 / 8 / 16 MHz. Results include `protocol` and
`clocksPerStrip` from that run, even if setup changes afterward.

Measured DMA completion includes driver/interrupt latency, and total time
also includes packing and task wake-up. This test does not measure CPU load,
SD bandwidth, FSEQ decoding, or maximum operating RPM. Rotation testing comes
after the stationary output passes.

## Check the real signals

Probe **CI and DI at an active strip connector**, referenced to board ground,
with equipment suitable for that connector's logic voltage. A scope shows
clock pulse widths and ringing; a logic analyzer can decode framing and lane
order. Start at 4 MHz. Expect clock periods of 250 ns, 125 ns, or 62.5 ns at
the corresponding selected rates. At 144 pixels, count 4,744 rising edges
in SK9822 mode or 4,712 in APA102 mode, with DI stable around each sampling
edge. Check all four lanes. For dark strips, first use the slow/static
**Wiring and signal check** on the same page to trace clock and data through
the buffers; successful DMA completion alone does not confirm strip output.

For SK9822, verify both high and low clock pulses meet the specified timing,
particularly at 16 MHz; measured duty cycle and PCB/level-shifter delays
matter. See the [SK9822 datasheet](https://cdn-shop.adafruit.com/product-files/2351/SK9822_datasheet_SHIJI.pdf).
Use the datasheet for the actual fitted LEDs if different.

## Driver and pin allocation

Active DI stays on **GPIO21, GPIO38, GPIO18, GPIO7**; shared CI stays on
**GPIO42**. Hall remains GPIO3 with one falling-edge pulse per revolution.
GPIO1 is assigned to motor speed PWM and is excluded from LED GPIO, DMA, and
signal checks. PSRAM and SD pins are untouched by the DMA routing.

The installed ESP-IDF 5.3 I80 driver requires eight data GPIOs and a DC GPIO,
even though this test has only four active lanes. Upper data lanes use the
existing unused output GPIOs **47, 48, 45, 39**, with every transmitted bit
zero; **GPIO40** carries DC with every phase level zero. The remaining
optional output data pins stay low. No new physical wiring is needed.
Expanding to sixteen active strips will need a sixteen-lane packer and a
revised peripheral allocation.

The DMA staging buffer and ISR context use internal RAM. The driver sends
data only (`lcd_cmd = -1`), with no command/dummy clocks, no bit/byte swapping,
and rising-edge sampling with the clock idle low. It uses the IDF-managed
LCD peripheral and GDMA channel through the
[Espressif I80 driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.3/esp32s3/api-reference/peripherals/lcd/i80_lcd.html).
The display task owns benchmark transfers and waits for completion before
reusing its buffer. HTTP controls remain available between samples.

API: `POST /diag/dma` with URL-encoded `mhz=4`, `mhz=8`, or `mhz=16` starts
the test; `GET /diag/dma` reads it; `POST /stop` cancels it. Repeated starts
and timing-counter resets during a test return HTTP 409. `/status` includes
the same data under `dmaTest`.
