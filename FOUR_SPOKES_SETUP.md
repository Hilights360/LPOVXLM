# Working four-arm sequence setup

The user confirmed that all four arms display the sequence correctly on
September 17, 2026, using native ESP-IDF firmware Ver1.2 Build 2. This setup
uses the existing firmware; no new firmware flash was required.

On September 18, **Ver1.2 Build 6** was installed and the user confirmed that
spoke widths are now consistent. Playback uses 8 MHz parallel DMA, prepares
frames outside the display lock, and stops on stalled SD reads. The geometry
and arm start channels below are unchanged. Brightness was subsequently set
by the user to 30%; the 10% entry below records the original baseline.
See [DMA_TEST.md](DMA_TEST.md) for the recovery checks and measured timing.

## Sequence and mapping

The current xLights export of `4_Spokes.fseq` contains one complete image per
frame: **80 virtual spokes, 144 RGB pixels per spoke, 34,560 channels**, at
40 frames per second for 30 seconds. The four physical arms sample this same
image at their respective angular positions.

| Setting | Working value |
|---|---|
| Sequence on SD | `/4_Spokes_80_POV.fseq` |
| LED protocol | APA102 / DotStar |
| Physical arms | 4 |
| Pixels per arm | 144 |
| Image spokes | 80 |
| First channel | 1 |
| Use individual start channels | Enabled |
| Arm starts 1, 2, 3, 4 | **1, 1, 1, 1** |
| Global / additional arm phases | 0 degrees |
| Brightness | 10% |
| Display duty | 60% |
| Angular strobe | Disabled |
| Sequence rate | 40 FPS |
| SD mode / maximum clock | 1-bit / 10 MHz |

The original physical arm geometry supplies the quarter-turn offsets. Viewed from the
ESP32 component side, rotation is clockwise and arm numbers run counterclockwise:
arms 1-4 use **0, -90, -180, -270 degrees**. A stationary Connector colors test
confirmed clockwise color order **red, white, blue, green**. The old geometry
assumed clockwise arm numbering and swapped opposite quarter colors between
successive arms. The user confirmed four steady quarters after reversing that
indexing on September 18, 2026. This correction applies to tests and playback;
the earlier symmetric four-spoke sequence did not expose the reversed order.

Build 33 adds **LEDs > LED wiring setup** so these directions can be configured
from either viewing side. Stop the rotor, press **Light arms in order**, and
watch arm 1 light red followed by arms 2-4 in white. Save the direction in
**The lights moved**, and set **The rotor normally spins** from the same viewing
position. Both selections persist in NVS. When the two directions agree the
arm offsets are 0, 90, 180, 270 degrees; when they differ the offsets are
0, -90, -180, -270 degrees. Changing viewing sides reverses both selections
and preserves the offsets. Existing settings initially retain the original
counterclockwise arm order and clockwise rotor direction. The rotation setting
describes the actual motion and does not drive the motor.

Build 33 was installed and its direction combinations and NVS persistence were
checked on September 18, 2026. Firmware, native geometry tests, and browser
tests passed; existing controller and SD preferences were retained. Records
are in `build/diagnostics/led-wiring-build33-20260918-172544/`. USB COM30 was
unavailable, so installation and live verification used Wi-Fi. The SD card
was already failing to mount before installation; the arm-order test operates
without it. The user then observed **clockwise** arm order from their viewing
position and had separately reported **clockwise** rotor rotation from that
side. Both settings are now saved clockwise, giving offsets **0, 90, 180,
270 degrees**; the light test stopped on save. This replaces the historical
counterclockwise arm-order assumption above. Playback with the new observed
order has not yet been visually checked.

Additional arm phases remain **0, 0, 0, 0**. Do not add
another quarter-turn to them. Each image spoke occupies
432 channels: spoke 0 starts at channel 1, spoke 1 at 433, and spoke 79 at
34,129. Every arm uses this same address calculation with its own angular
spoke index. Starts of 1, 433, 865, and 1297 instead shift the arms into
different image columns and run beyond the image at its end.

This profile is specific to the 80-spoke export. Older files with about
17,280 channels are different exports and must not be played with these
geometry settings without checking their model and channel offsets.

## 160-spoke stationary-image pattern test

On September 18, Build 33 ran the built-in alternating RGBW spoke pattern at
**160 spokes, 100% duty, strobe off**, with four 144-pixel arms and the user's
10% brightness / center fade settings retained. Both observed arm order and
rotor direction remained clockwise. The user confirmed **steady and evenly
spaced** bands. These settings and the alternating pattern were left active.

The first 12 samples reported about 123-125 RPM, no timing-limited samples,
and no LED transfer errors, but counters recorded three missed spokes and
359 skipped arm paints. A subsequent 16-sample observation included an isolated
2030.5 RPM reading and one timing-limited sample, with increases of 106 missed
spokes and 1751 skipped arm paints. Mean LED output time was about 861 us,
maximum 2124 us; the final separate RPM reading was 133.2. The visual result
does not establish a clean timing trace or validate 160 spokes at 1400-2000 RPM.
The cause of the abrupt RPM readings has not been determined.

For the planned matching xLights export, use **160 image spokes x 144 RGB
pixels = 69,120 channels per frame**. All four arm starts remain **1** because
the arms sample the same full image. The existing 80-spoke file is unchanged
and requires an 80-spoke setting if played again. The SD card was still
unmounted, so this test did not exercise sequence reading or decompression.
Evidence: `build/diagnostics/spokes-160-20260918-173741/`.

## Test4 upload and corrected 160-spoke export

On September 18, the user reported trouble uploading
`F:\Dropbox (Personal)\All Projects\LPOV\FSEQ\Test4.fseq`. The controller log
showed that the original 98,518,088-byte upload had completed. That export and
the xLights Spinner model contained **190 spokes**, while the controller was
configured for 160. The user corrected the model and re-rendered the sequence.
The corrected 82,966,083-byte `Test4.fseq` then uploaded successfully through
the browser. The controller's synchronous file transfers delay other HTTP
requests while running; the current upload page has no progress percentage.

A separate **`/Test4_160_POV.fseq`** was prepared and uploaded for playback:

- 160 spokes, 144 RGB pixels, 69,120 channels, 1200 frames, 25 ms/frame (40 FPS).
- Lossless zlib compression with one frame per block: **20,134,822 bytes**.
- Every decoded frame matched the corrected source, and the firmware's native
  FSEQ metadata parser accepted the resulting file.
- The complete uploaded file was downloaded and matched SHA-256
  `2219e1d6f265e688ee7b86127a5cf60589fb89700d33a44d4ec0b9a08bb924eb`.
- No new SD errors occurred during upload or readback at 1-bit/20 MHz.
- Controller settings were preserved: 160 spokes, 144 pixels, 40 FPS, all arm
  starts at 1. Sequence playback itself was not started during verification.

The compressed copy reduces transfer size and SD read demand. The original
corrected xLights file was left unchanged. Local copy and evidence:
`build/diagnostics/test4-upload-20260918-175721/`. The original 190-spoke file's
readback was cancelled after about 60 MiB to let the user's corrected upload
proceed; that obsolete file was not fully verified.

## Prepared 80-spoke playback copy

`4_Spokes_80_POV.fseq` is a losslessly repacked copy of the current export,
using FSEQ v2 zlib compression with one frame per compression block. It
contains the same RGB values and frame timing as the source. The original
xLights project, source sequence, and older SD sequence were not replaced.

The prepared file is 2,038,478 bytes instead of 41,494,394 bytes. Every one of
its 1,200 decoded frames was compared with the source before upload, and the
complete uploaded file was downloaded again and matched byte for byte.

SHA-256 of the prepared file:

```text
fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81
```

The local prepared copy and verification records are in `build/diagnostics/`.
They are generated artifacts; preserve the SD copy if cleaning the build
directory.

## Problems corrected during setup

- Arm start channels were offset into different slices of a shared image.
- The selected SD sequence and configured image spoke count did not match.
- The 1% display duty requested a lighting window far shorter than the
  roughly 2.1 ms GPIO transfer time. The working baseline uses 60% duty.
- Repeated SD read failures in 4-bit mode caused playback failures and an
  empty directory listing. Switching to 1-bit restored file access. This
  establishes a working configuration, not the physical cause of the
  4-bit failures.

The user confirmed the resulting image on the physical spinner. Higher RPM,
other exports, and other SD modes require their own timing and visual checks.
