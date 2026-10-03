# LPOVXLM

Native ESP-IDF firmware for the ESP32-S3 N16R8 persistence-of-vision controller.
The current PCB has sixteen optional outputs; this build activates the four
quarter-turn connectors and uses one falling-edge Hall pulse per revolution.

The active application is in [`main/`](main). It uses ESP-IDF directly, with no
Arduino component or Arduino libraries. The original root-level `.ino` and
Arduino `.cpp` files remain as migration references and are excluded from the
ESP-IDF build. Shared wiring is in [`BoardPins.h`](BoardPins.h).

The SD connector close-up establishes **CLK=11, CMD=12, D0=10, D1=9,
D2=14, D3=13** (GPIO numbers). There is no configured card-detect input;
mounting probes the card over SDMMC. GPIO14 is DAT2 and must not gate mounting.
After installing this build, use **SD > Retry mount**. For a conservative test,
select **1-bit**, **400 kHz**, then **Save and mount**, and check the logs for
the GPIO map and mount result. Mount failures never trigger formatting.

If the log reports `sdmmc_init_ocr` / `ESP_ERR_TIMEOUT`, initialization failed
before the filesystem was read. The September 18 SD close-up shows the correct
socket **P4=3.3V, P5=CLK** order; the earlier suspected swap does not apply to
that image. With power disconnected and the card removed, verify socket power,
ground, and signal continuity on the assembled board. Then fully remove board
power (including USB), reseat the card, and retry at 1-bit/400 kHz. Each attempt
logs the idle CMD and DAT0-3 levels with internal pull-ups enabled; all should
normally read 1.
A LOW helps narrow the electrical checks, but all HIGH does not prove socket
continuity, external pull-ups, or card power. The PCB still needs its external
10k pull-ups on CMD and all four DAT lines even in 1-bit mode; see
[Espressif's SD pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/v5.3.3/esp32s3/api-reference/peripherals/sd_pullup_requirements.html).

The September 18 [4-bit investigation](SD_TROUBLESHOOTING.md) reproduced failures
on two cards. Continuity checks passed. Build 30 subsequently passed 120 MiB
of hashed read-only tests, while 4-bit write/read benchmarks still failed with
CRC errors at 4 and 10 MHz. The fault can affect writes as well as reads.
The SDK's divider selection reduces the nominal output phase delay at those
intermediate clocks. A 20 MHz comparison passed while 10 MHz failed, followed
by 32 consecutive 16 MiB tests at 4-bit/20 MHz with no errors (512 MiB written
and read/verified). The report includes the evidence and remaining limits.

## SD pin scope

**SD pin scope** in the main navigation (`/sd/scope`) is a permanent diagnostic.
**Build 44 is installed**, with the link on Controller, LEDs, Speed test,
Wi-Fi setup, and SD Card, plus return links from the scope page.
It is also linked from **SD Card > Diagnostics > Open SD pin scope** and uses the ESP32's
ADC with the card left inserted. **All six pins** is the default: one job
captures 1,024 readings each from DAT0, DAT1, CMD, DAT2, DAT3, and CLK, and
shows all six traces together with a comparison table. Single-pin captures
remain available. Requested spacings are 100 us, 1 ms, or 10 ms. Samples keep
actual timestamps and gaps; each pin has a start offset within the set.

The six captures are sequential, not simultaneous. The display aligns each
pin's own time zero for comparison; the table and CSV/JSON exports preserve
the start offsets. Zoom and position apply to all six traces. Automatic units
use raw counts for the whole set when any pin has no valid voltage readings;
Voltage where valid shows gaps/over-range markers instead of invalid voltages.
The complete set remains available across page reloads until another SD tool
starts or the controller restarts. CSV export includes all 6,144 samples.

**Wi-Fi during capture > Off, then restore** stops both the access point and
router radio links before sampling, waits 100 ms for settling, and restarts
Wi-Fi afterward. Samples remain in RAM while the page is disconnected. The
page reconnects automatically; if the computer switches networks, reconnect
to POV-Spinner. Cancellation is unavailable while offline. The fastest all-pin
capture keeps the radio off for about 1.6 seconds; reconnecting can take longer.
Slow 10 ms sampling can take about 65 seconds. A 90-second limit restores the
radio if the capture worker does not finish. Captures record whether Wi-Fi was
confirmed off, restart success, and stop/resume offsets. Export filenames also
identify the Wi-Fi condition. The selection does not change saved Wi-Fi settings.

Capture stops playback, waits for outstanding reads, detaches the SD host
once for the entire set, and switches one pin at a time to analog input.
Cancellation preserves completed and partial traces and always restores the
SD pads. A previously mounted card is remounted at its previous width and
clock, without fallback or a saved-settings change; an unmounted card stays
unmounted. Restoration errors retain the data and direct users to Retry mount.

This is a **slow voltage diagnostic with SD communication paused**, not an
oscilloscope for active SD transfers. It cannot resolve 20 MHz edges, ringing,
or short glitches. The nominal ADC range ends around 3.1 V; normal 3.3 V HIGH
levels and their noise may clip. Over-range samples keep raw counts and an
explicit flag, with no numeric voltage. ADC sampling adds loading and noise;
the recorded range is not an isolated measurement of board noise. DAT0/1 use
ADC1; CLK/CMD/DAT2/3 use ADC2, shared with Wi-Fi. Missed readings are gaps,
not zero volts. The selected pin's internal pull-up is disabled during
sampling; the five external pull-ups on CMD and DAT0-3 remain connected.
**CLK / GPIO11 has no external pull-up.** The scope detaches the SD host and
leaves CLK as an input without an internal pull-up, so it has no driven HIGH
level during capture. Its falling/low ADC trace is consistent with that
undriven line and is not, by itself, evidence of an SD clock fault.

**Build 42 added Wi-Fi-off sampling.** Two Wi-Fi-on/off pairs each captured all 6,144 raw
readings. CMD and DAT0-3 dipped with Wi-Fi on and remained at 4095 in both
radio-off captures. Saturation prevents interpreting this as clean voltage or
measuring dip amplitude; the comparison implicates Wi-Fi activity in the
recorded pattern without distinguishing supply/pin disturbance from ADC effects.
CLK remained variable with Wi-Fi off. Both links and SD access recovered after
capture and a cancellation check. Settings and root file metadata were preserved,
with no new SD errors, and a 2,038,478-byte download matched its reference SHA-256.
The final radio-off set remains in RAM; the live page displayed/exported all six
traces and preserved the condition labels across reload and mobile layout checks.

Validation includes the ESP-IDF build, existing web smoke tests,
`node tests/sd_scope_page.cjs`, and installed-page capture/export checks.
Evidence: `build/diagnostics/sd-scope-install-20260921-071005/`, including the
four captures, comparison summary, and `wifi-on-off-comparison.png`.
Electrical accuracy still requires an external reference. See
[Espressif's ADC documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.3/esp32s3/api-reference/peripherals/adc_oneshot.html)
and [ADC range guidance](https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/adc.html).

## Build in this repo

ESP-IDF **5.3.3** is installed on this computer. From PowerShell:

```powershell
.\build.ps1
```

The helper loads the ESP-IDF environment and builds into `build/idf/`.
The application binary is **`build/idf/lpovxlm.bin`**. The settings select
ESP32-S3, 240 MHz CPU, 16 MB QIO flash at 80 MHz, and octal PSRAM at 80 MHz.
The existing [`partitions.csv`](partitions.csv) retains NVS and two OTA slots.

Every page header shows **LPOVXLM Spinner — Ver1.2 · Build N**, read from the
running firmware. Each application build increments a shared local counter
in `build/firmware_build_number.txt` and refreshes the embedded build time.
Failed builds can consume a number. Deleting that counter resets numbering.
The helper prints the exact OTA path, version, number, and local timestamp.

The header stays visible while scrolling on Controller, LEDs, Speed test, and Wi-Fi setup,
including on phones. Its **Reboot** button stops the lights and restarts the
controller. Saved settings are retained; a running LED test must be started
again after reboot. Section links and notices remain below the header.

The **SD Card** page (`/sd`) contains card setup and mount controls, files,
diagnostics, firmware updates (including OTA), and controller logs. Diagnostics
and direct OTA updates are available even when the SD card is not mounted.
Older Files, Updates, OTA, and Logs links lead to the corresponding section.
Files can be sorted by name (natural, case-insensitive order) or modified date,
in either direction, with folders first and unknown dates last. The file sort
choice is remembered in the browser. Uploads preserve source modified dates;
older files display the timestamp stored on the card. The existing 512-entry
directory limit still applies. Wi-Fi scan results also offer name sorting,
and strip-speed observations can be sorted by recorded date or clock rate.

**Automatic SD recovery** is enabled by default under **SD Card > Card setup**.
New settings prefer **4-bit at 20 MHz**, with **1-bit at 20 MHz** as the first
backup and **1-bit at 400 kHz** as the last resort. Automatic recovery skips
the problematic 1/4/8/10 MHz clocks. Older saved preferences at those retired
rates normalize to 20 MHz; the saved bus-width preference is retained.
Selecting 40 MHz adds a 4-bit/40 MHz attempt before the default ladder;
explicit 1-bit/40 MHz uses 1-bit at 40 MHz, 20 MHz, then 400 kHz.
Selecting 400 kHz tries only that rate, with 1-bit backup if 4-bit is selected.
Disable automatic fallback to try only the selected setting.
The page shows the actual clock and width in use, plus recovery progress.

The separate **Recover after SD read/write errors** option steps down after
playback stalls/repeated I/O errors, failed upload/download I/O, or speed-test
I/O/verification failures. It stops playback, drains outstanding reads, then
remounts in a worker so web controls remain responsive. Recovery starts below
the failed profile and never loops back to faster settings. Missing files,
full cards and invalid FSEQ contents do not trigger a speed reduction.
**Try next lower setting** exercises this recovery path manually; **Retry
mount** starts from the preferred setting again. Interrupted transfers must
be retried. Recovery never formats a card or retries destructive operations.
Remount success alone does not verify data transfers: run the speed test at the
new setting. The SD page retains the last SDMMC transfer error, command, and
bus setting across remounts, distinguishing CRC/timeouts from filesystem errors.
The full status also retains the eight most recent errors so cleanup failures
do not hide the original error.

**SD speed test** writes and verifies a temporary 1, 4, or 16 MiB file using
16 KiB blocks at the currently mounted bus width/clock. Results include read
and write MiB/s, worst block pauses, and verification. Write timing includes
flush/sync; pattern generation, verification and cooperative delays are excluded
from the I/O rate. The worker removes only its exclusively created temporary
file and reports cleanup failures. Failed tests show partial byte counts and
retain the original I/O error ahead of any close/cleanup errors. Cancellation
and the 60-second deadline are checked between SD operations. Playback and other storage operations pause
while the tool runs; the web status and diagnostic pages stay available.

**Format SD card** requires typing `ERASE`. It replaces all card partitions
with one FAT filesystem, then remounts at the saved SD settings. ESP32/NVS
settings are preserved; sequences and the SD settings backup are erased.
Formatting can initialize a card whose filesystem does not mount, but cannot
repair an electrical connection or a card that fails SD initialization. Normal
mount retries never format automatically. Formatting cannot be cancelled once
started. Do not remove power or the card during the operation.

Build 18 adds **SD Card > Diagnostics > Saved crash report**. Firmware panics save an
ELF core dump in the existing 64 KiB flash partition (up to four tasks, with
a separate 2 KiB capture stack). `GET /diag/crash` returns the saved task,
exception, backtrace, and application ELF hash; `GET /diag/crash.bin` downloads
the raw dump for ESP-IDF decoding with the matching application ELF. This is
diagnostic support, not a fix for an unidentified crash. Earlier firmware did
not retain these reports, so an earlier panic cannot be reconstructed from it.

Use **Speed test** (`/speed-test`) to inspect sustained LED patterns at 2, 4, 8,
10, 16, 20, 26.67, or 40 MHz. Runs end automatically and offer visual pass/error
recording and downloadable results. Start at 4 MHz with the rotor stopped;
the ESP cannot determine whether the LEDs display correctly. This test leaves
normal playback at 20 MHz and preserves saved brightness, center fade, and duty.
See [`DMA_TEST.md`](DMA_TEST.md) for test behavior and timing interpretation.

Plain `idf.py build` without `-B` creates **`build/lpovxlm.bin`** instead.
That file and **`build/idf/lpovxlm.bin`** are separate outputs: use the path
reported by your build command. Both build folders use the same number counter;
each has its own `generated/build_info.json` describing its application.

Other commands:

```powershell
.\build.ps1 -Action menuconfig
.\build.ps1 -Action size
.\build.ps1 -Action flash -Port COM29
.\build.ps1 -Action monitor -Port COM29
```

`build` does not upload. `flash` builds and uploads the bootloader, partition
table, OTA metadata, and application. `monitor` opens UART at 115200 baud;
exit with **Ctrl+]**. COM29 was the connected CH343 USB serial adapter during
conversion; specify another port if Windows assigns a different one.

Set `IDF_PATH` or pass `-IdfPath` to use a different ESP-IDF 5.3.x installation.
In an already activated ESP-IDF terminal, the equivalent is:

```text
idf.py -B build/idf build
idf.py -B build/idf -p COM29 flash monitor
```

VS Code's ESP-IDF extension is configured to use `build/idf`. Generated
`sdkconfig`, build output, and downloaded managed components are ignored;
`sdkconfig.defaults` and `dependencies.lock` define the reproducible defaults
and dependency versions. The only external component is Espressif mDNS.

## Web pages and operation

After flashing, connect to **POV-Spinner** (password **POV123456**) and open
**http://LPOV.local/**, or **http://192.168.4.1/** on the controller's access
point. The default mDNS hostname is `lpov`; the firmware supplies `.local`.
On upgrade, this device's previously generated `pov-xxxxxx` hostname changes
to `lpov`; custom saved names are retained. To restore the default, clear
**Controller hostname** on the Wi-Fi setup page and save. The IP address is
available if the client network does not resolve mDNS.

The pages in [`main/web/`](main/web/) are embedded in the firmware and served
by `esp_http_server`; no SD card is needed to open them:

- Playback, brightness, duty, frame rate, geometry, and per-arm mapping.
- Wi-Fi, SD bus settings, background playback, angular strobe, and phase.
- File browsing, upload, download, rename, delete, and folder creation.
- One dedicated LEDs page with setup, light tests, signal checks, and DMA timing.
- All-arm color fade: every pixel on every active arm shows the same color,
  fading together through a repeating 12-second color cycle.
- Native OTA upload and SD `firmware.bin` installation on restart.

**Use FSEQ settings (FPS and spokes)** beside the playback controls reads each
successfully opened file automatically. Frame timing uses the exact FSEQ
millisecond interval: 25 ms is 40 FPS, 50 ms is 20 FPS, and 33 ms remains exactly
33 ms rather than being rounded to 30 FPS. A missing interval or one faster than
the supported 120 FPS falls back to manual timing with an explanation.

FSEQ has no standard spoke-count field. For a file containing one complete RGB
spinner image, the firmware infers **spokes = stored channels / (pixels per arm
× 3)**. All active arms must use the same start channel, matching the start of
the file's contiguous channel span. For example, 110,592 channels and 144 pixels
give 256 spokes; 69,120 channels give 160 spokes. Physical arm count does not
divide the image width. Contiguous sparse ranges also work; sparse gaps,
separate arm streams, a mismatched channel start, or incomplete RGB rows retain
manual spokes. Files containing unrelated models need manual setup even if
their channel count happens to divide evenly. The page labels the inference
and any fallback. See the [FPP format specification](https://github.com/FalconChristmas/fpp/blob/master/docs/FSEQ_Sequence_File_Format.txt).

The checkbox persists across restarts and is initially off on upgrade. Manual
FPS and spokes remain saved separately and return when it is unchecked. Fields
supplied by the file are read-only; fallback fields remain editable. Detected
spokes also feed Auto-calc duty and rotation stability tests. The most recently
loaded file's metadata is retained for those tests until another sequence is
opened or the controller restarts. A failed open clears the previous metadata.
`/status` and `/diag/timing` expose `fileSettings` with the effective values,
file interval, inferred spokes and explanation; `settings.fps` and
`settings.spokes` continue to report the saved manual values.

**Reverse radial pixel order for this sequence** saves a separate mapping for
each sequence path in controller flash. Normal order sends the first RGB pixel
to the hub; reversal sends it to the tip. Changing files restores that file's
mapping, and the hub brightness fade remains anchored to the physical input.
This setting is independent of automatic FPS/spoke detection. FSEQ does not
identify which end of a physical strip its first channel represents.

The supplied older `Test4 80 Arm 20FPS.fseq` and current `Test4.fseq` contain
opposite radial orders despite both effects being outward shockwaves. Analysis
of the former places the light near source pixel 143 first and pixel 0 last;
the latter advances from pixel 0 toward 143. A blanket PCB reversal cannot
display both correctly. Build 63 replaces Build 62's temporary blanket reversal
with the per-file mapping. The current file's pixels were checked against
the actual controller, and the older SD file's SHA-256 matched the analyzed
local export. Native regression checks both orders through all four
packed LED lanes and verifies that center dimming stays at the hub.

Build 63 also reads contiguous RGB rows once per arm instead of repeating the
channel lookup for every color. Sparse rows that cross a gap retain the normal
per-channel path. This reduces preparation work within the spoke deadline.
Investigation and installation evidence:
`build/diagnostics/radial-mapping-20260922-155841/`.

Build 63 is installed. The older 80-spoke file is saved as tip-first and
`/Test4.fseq` as hub-first. Both mappings were verified on all four outputs
after switching files and after a controller restart. The current sequence
was resumed with the user's latest manual 20 FPS settings unchanged.

Build 54 is installed with Use FSEQ settings enabled. Native and browser tests
passed, including exact 33 ms timing and ambiguous-layout fallback. Live checks
switched between the existing 80- and 160-spoke files at 40 FPS and the
256-spoke Spooky file at 20 FPS, verified channel mapping and manual fallback,
and confirmed the checkbox survives a restart. The Spooky sequence was restored;
all prior manual settings and SD root file metadata were preserved. Evidence:
`build/diagnostics/fseq-auto-install-20260922-072905/`.

**Auto-calc lowest duty for RPM** is beside Display duty on Controller and LEDs.
Checking it saves the option and continuously selects the smallest whole-percent
duty that fits the current revolution period, image spoke count, and LED transfer
budget, including a wake-up margin and room for the following black transfer.
When all arms share spoke boundaries, the calculation uses the faster cached
black-frame transfer. Staggered arm timing retains the full color-transfer
budget for blanking, because other arms can still be lit.
The controller does this even with the browser closed. It uses recent transfer
peaks (approximately two seconds) with a conservative pixel/protocol estimate
as a floor, so an isolated slow transfer can recover instead of raising duty
for the entire session. Dynamic duty updates stay in RAM; only the checkbox
and manual settings are saved to flash.

Build 60 gives manual duty the same budget. Both paths now call
`paintTransferBudget()`, which sizes the reservation from the actual 20 MHz
parallel DMA wire time plus the packing pass, and raises it only through the
two-second transfer peak. Manual duty previously used `output.stats.maxUs`, a
lifetime maximum that never decayed, over a floor of
`frameBytes + pixels x 3 + 250` microseconds left from bit-banged GPIO output.
For four 144-pixel APA102 arms that floor reserved 1271 us against a real wire
time of 4712 clocks (about 236 us), and one stalled 2225 us transfer pinned the
budget at 2425 us for the rest of the session. `spokeGate()` then refused to
paint whenever a spoke was shorter than the budget, so at 256 image spokes the
lights went dark with "Spoke timing is too short" above roughly 97 RPM. The new
estimate is 874 us, which reaches about 268 RPM at 256 spokes, and it recovers
after a slow transfer instead of latching. Measured on the installed controller:
mean transfer 504 us, minimum 266 us.

Build 61 fixes a second Auto-calc failure: a feasible minimum duty could still
produce black output because the display reserved the entire paint budget
again before each arm. Preparation already completed was charged repeatedly,
while the calculation measured only packing/transmission and allowed just
100 us for task wake-up. An 80-spoke sequence accumulated hundreds of rejected
arm paints per second with no timing-limit warning.

The deadline checks now retain one reservation across all four arms and check
only the remaining submission work after preparation. Recent paint timing
includes pixel preparation through completion, and task wake-up has a separate
250 us allowance. A preparation overrun feeds back into the next calculation
even when that paint is discarded. The display still rejects a transfer that
cannot finish before its deadline. `/diag/timing` includes preparation times
and separate color/black transmission counters to distinguish active output
from a sequence clock advancing while all LEDs are blank.

Native deadline regression tests and the firmware build passed. Build 61 was
installed with saved settings unchanged, Auto-calc enabled, and the user's
`/Test4 80 Arm 20FPS.fseq` resumed. Rotation was stopped at installation, so a
live rotating-output check is pending. Evidence:
`build/diagnostics/auto-duty-blank-20260922-154549/`.

The live duty field follows the calculation and manual editing is disabled while
Auto-calc is checked. It takes precedence over angular strobe during playback,
Quarter colors and Alternating spokes. Unchecking restores the saved manual duty
and strobe. Brightness, sequence speed, arm offsets and geometry remain as set;
the level pattern retains its fixed widths. The option starts off on upgrade.

When paint and black transfers cannot both fit, Auto-calc reports **100%
continuous output** if a complete image update still fits. If even that cannot
fit, rotation output stays dark with a message to reduce RPM or image spokes.
Without magnetic pulses it waits in darkness. These are timing-based estimates;
the controller cannot judge the appearance of the physical LEDs. `/status` and
`/diag/duty` include `dutyControl` with the calculated duty, feasibility, transfer
budget, and continuous-output flag. `settings.duty` remains the manual value.

**LEDs > Short flash verification** provides a temporary 30-second comparison
on the existing four 144-pixel APA102 outputs while a sequence is playing.
The test sends one to three identical color frames followed by a cached black
frame in one DMA transaction at 20 MHz, removing the separate software wake-up
between color and black. **Sharp**, **Balanced**, and **Fuller** give nominal
per-pixel color-to-black spacing of 235.6, 471.2, and 706.8 microseconds. Balanced
is the default. The complete bursts have 471.2, 706.8, and 942.4 microseconds of
wire time respectively. These are calculated timings, not optical measurements.
Wider flashes fill more of a spoke at the expense of edge sharpness. A prepared
burst that cannot finish before its spoke boundary is discarded before
transmitting any color.

The test requires synchronized arm spoke boundaries, uses the saved brightness,
temporarily overrides duty/strobe, and returns to normal timing after 30 seconds
or **Return to normal timing**. Stop lights, a new sequence, or another light
test also cancels it. No settings are persisted. `POST /diag/flash-proof` with
`enable=1` starts and `enable=0` cancels. Optional `colorFrames=1`, `2`, or `3`
selects the flash width; omission retains the current choice. The page applies
width changes immediately during a test. This choice is temporary and defaults
to Balanced after reboot. `/status` and `/diag/timing` include
`flashProof` with its state, remaining time, brightness, nominal pulse, completed bursts,
and late prepared pulse count. The diagnostic nominal hold time is flagged
with `holdDurationIsNominal` while the test runs.

The Build 54 baseline near 95 RPM showed Auto-calc sometimes selecting 100%
continuous output and a maximum software blank-start delay of 5,635 microseconds.
An impossible 25,963 RPM reading was also captured. Baseline captures and the
bounded flash-test evidence:
`build/diagnostics/blanking-verify-20260922-095153/`.

Build 55 introduced this test. Native protocol/deadline checks and browser tests passed.
A 30-second live run completed 7,590 paired color/black bursts at approximately
90-91 RPM during valid samples, discarded 238 late prepared pulses, and returned
to the original sequence and timing with saved settings unchanged. Hall outliers
were also present. After repeating the comparison, the user confirmed the image
was dim but "MUCH BETTER image quality wise." This is a visual pass for the short
flash approach on the existing four 144-pixel arms. That initial test was capped
at 10%. At the user's request, short flashes now follow the normal brightness
control while retaining the same pulse width. Optical pulse width has not been
measured directly.

The subsequent Test4 blackout investigation captured a 49,781-microsecond Hall
interval (1,205 RPM) while the rotor was near 87 RPM. Auto-calc selected zero
duty for that false speed. The Hall filter now rejects intervals shorter than
333,333 microseconds, allowing 180 RPM for this rotor's stated 150 RPM maximum
with one index per revolution. Rejected pulses change neither phase nor speed.
Speed and spoke timing use the mean of the last three accepted revolution
intervals; each accepted sensor edge still establishes the physical index.
After a gap longer than three previous revolution intervals and one second,
the filter restarts acquisition without averaging the stopped time into speed.
`/status` and `/diag/timing` expose `hallFilter`, including the average, latest
accepted interval, sample count, rejected pulse count, and last rejected interval.
Native tests cover the captured glitch, rolling averages, 100/150 RPM operation,
the rejection boundary, and stop/restart. This fixed speed ceiling must be revised
if the hardware is changed to run faster or provide multiple indexes per turn.

Build 56 was installed with the Hall filter and three-revolution average. All
native suites passed, and saved settings were preserved. Across 36 one-second
Test4 status samples, RPM stayed between 89.25 and 90.37, rejected pulses rose
from two to eight, and no sample selected zero duty or reported an SD stall.
The user still reported excess darkness in normal playback and confirmed that
Test4's effects include intentional dark gaps. After short-flash playback was
restarted, the user confirmed the effects looked much better and requested more
brightness. Build 57 makes short flashes follow the saved brightness control,
starting at the existing 50%, with the same nominal 235.6-microsecond pulse.
The build and LED-page browser checks passed. The installed controller reports
50% short-flash brightness and retained all saved settings. A local comparison
helper renews the temporary test while Test4 keeps playing; cancellation or a
playback change ends that helper. Captures are alongside the flash-test evidence.

Build 58 adds the flash-width control, caches the radial brightness gains, and
packs RGB lanes directly with fixed bit shifts. Native tests compare the packed
stream against the original byte encoder across both protocols, all brightness
extremes, and multiple strip lengths; cached fades are compared with the original
calculation, and every flash width is checked at the timing deadline. All native
suites and LED-page browser checks passed. Build 58 was installed with saved
settings preserved, then Test4 resumed at 50% brightness with Balanced flashes.
Across about 87 seconds near 93 RPM, 28,855 paired color/black bursts completed
and 669 prepared bursts were discarded as late (2.27% of prepared colored
bursts). This is similar to the earlier narrow-flash discard rate despite twice
the nominal lit duration. Eight sampled completed color bursts averaged 297
microseconds of packing, including the repeated color frame. Physical refresh
is still limited to four arm passes per revolution; optical gap/blur improvement
requires the user's visual comparison.

Build 53 introduced Auto-calc. Native and browser tests passed, and a live playback
check selected 46-49% duty at approximately 110 RPM with 160 image spokes and
144 pixels per arm. The enabled option survived a restart. Auto-calc was then
turned off and the original settings and `/Big_Eye.fseq` playback were restored;
SD root file metadata was unchanged. This validates control/timing behavior,
not visual LED quality. Evidence:
`build/diagnostics/auto-duty-install-20260922-064037/`.

**Animation frames change only between complete image sweeps.** Previously,
the decoder replaced the displayed image on each FSEQ frame deadline, even
while the arms were partway through drawing it. Expanding rings could therefore
have several different radii around the circle, producing stair steps.

Playback now keeps separate displayed, pending, and SD-read buffers. The
decoder follows the selected file/manual frame rate and queues the latest due
image; the display adopts it at a sweep boundary and holds it throughout that
sweep. A late read repeats the current image. Intermediate animation frames
can be skipped so the sequence keeps its intended duration.

With four quarter-turn arms, equal additional arm phases, and a spoke count
divisible by four, a complete image takes a quarter revolution. At 120 RPM
that is **8 complete images per second**, even for a 40 FPS file. Other arm
counts, unequal phase corrections, or other spoke counts conservatively hold
each image for a full revolution. Frame selection follows the global phase
and arm 1 correction. Stopping rotation falls back to clock-based progression;
pause holds the displayed frame. The `frame` status value is the image actually
selected for display, while the frame-read counter tracks decoding.

This removes frame changes within a sweep; motion smoothness is still limited
by the physical image rate, and a camera exposure can capture multiple sweeps.
The native expanding-ring regression reproduces 16 torn images with the old
timing and zero with sweep latching, including reversed arm order and delayed
reads. Hardware visual confirmation is still required for this change.

Build 59 was installed through the controller's access point on September 22,
2026. The controller restarted with saved settings unchanged and the SD card
mounted at 4-bit/20 MHz. `/Test4.fseq` is playing with 256 spokes and its 25 ms
file interval; three observations showed displayed frames 85, 175, and 266 at
about 93 RPM, with no reported playback errors, timing warnings, or SD stalls.
This confirms installation and progression, not the visual ring shape.
Evidence: `build/diagnostics/playback-sweep-build59-20260922-130751/`.

The **Loop playback** checkbox beside the playback controls saves automatically
and persists across restarts. It starts enabled to preserve repeated playback.
Uncheck it to finish the current pass, turn off the LEDs, and show **Finished**.
The last frame is retained through a complete sweep before playback stops;
delayed reads cannot wrap a single pass back to the beginning. You can change Loop while playing or
paused. A completed single pass stays stopped, including when idle autoplay or
background playback is configured, until another playback/test action is taken.

Build 34 was installed with this control. Native and browser checks passed;
a temporary one-second black sequence verified single-pass completion, repeated
looping, disabling Loop during playback, and pause/resume on the controller.
The unchecked setting survived a reboot. Loop was left enabled, other settings
and original files were preserved, and the temporary file was removed. Evidence:
`build/diagnostics/playback-loop-build34-20260918-181853/`.

The **Recent files** dropdown beside Sequence path fills the path without
starting playback. It remembers the last 12 selections, successful plays, and
sequence uploads in this browser, and includes up to 12 other root-folder FSEQ
files ordered by their SD modification dates. The refresh arrow reloads the
card list without replacing a typed path. Manual paths still work for any
folder; use the SD Card page to browse folders. History survives page reloads
and controller restarts in the same browser and address. Successful SD-page
renames/deletes update history; unavailable or busy storage leaves cached
choices and the playback controls usable.

Build 35 includes the picker. Browser checks and the firmware build passed;
the installed controller listed `Test4_160_CW_POV.fseq`, retained its selection
across a page reload, and preserved controller settings with no new SD errors.
Evidence: `build/diagnostics/recent-sequences-build35-20260918-183915/`.

**Wi-Fi setup** opens its own page at **http://192.168.4.1/wifi**. Press
**Scan networks**, select an SSID from the signal/security list, enter its
password, and choose **Save and connect**. Hidden networks can be entered
manually; an **Open network** checkbox explicitly clears the password.
Both router and access-point password fields have **Show / Hide** controls.
Show can retrieve the saved router password on request; normal status updates
and scan results do not contain passwords. The access-point credentials are
displayed read-only.

Open **LEDs** at **http://LPOV.local/leds** for setup and all LED tests.
**Arm offset and level** (`/leds#alignment`) adjusts all arms together relative
to the magnetic sensor. With the unit rotating, press **Show level pattern**:
one half of the disk is green and the opposite half has three red spokes.
Use **Rotate clockwise / counterclockwise** until green is on top, its edge
is level, and the middle red spoke points straight down. Choose a 0.1, 1, 5,
or 15 degree step, or enter an offset and press **Save offset**.

The buttons apply and save immediately, including during playback. The offset
survives restarts and is the existing global phase setting, so previous
calibration is retained. Positive degrees shift the image against rotation;
the direction buttons use the rotor direction saved in **Wiring setup**, viewed
from that same side. **Reset offset to 0** restores the sensor reference without
changing individual arm trims. Controller rotation timing links to this section.

The level pattern uses a continuous 180-degree green region and three 9-degree
red rays centered at 225, 270 and 315 degrees in the image coordinate system.
It shares playback's arm geometry, global offset, individual trims, brightness,
center fade and 20 MHz output. Its fixed angular windows ignore display duty,
strobe and image spoke count without changing those saved settings. It replaces
playback, needs no SD file, and stays dark without magnetic pulses. **Stop level
pattern** ends it; the saved offset remains active for subsequent playback.

The **LED setup** section selects **SK9822** or
**APA102 / DotStar**, active arms (1-4), and pixels per arm. The old firmware
used Adafruit DotStar with BGR wire order and described the strips as
SK9822/APA102; it does not establish the actual fitted LED part. Both choices
use separate clock/data and BGR order. SK9822 remains the default, preserving
the previous framing. Settings persist across restarts. Old `/setup` links
redirect to the setup section on the LEDs page.

On the same page, set brightness to 10% and
start with **Solid red**, or use **All-arm color fade** for a synchronized
12-second color cycle. The page also has Arm RGB, connector colors, Hall,
DMA, and output diagnostics. **Quarter colors** and **Alternating spokes**
check spatial alignment using the configured spoke count, brightness, and
duty. These two tests require rotation and magnetic pulses; the other light
tests work stationary, with the Hall test following its sensor. No LED test
requires an SD file. **Stop lights** ends the selected test.
Zero brightness keeps lighting tests dark.

**White blinking** switches all active arms between white and black every
500 ms (one complete cycle per second), using saved brightness and center
fade. It runs while stationary and remains active until Stop lights or another
operation replaces it. Build 44 adds this button and a read-only SD load-test
option: start the pattern with `POST /led/blink`, then send
`POST /sd/read-test` with the normal file/size/block parameters and `keepBlink=1`.
The read worker retains this independent LED pattern while exclusively owning
the SD file. Other SD operations and ordinary read tests still stop the lights.
The option rejects an inactive/dark blink, and Stop lights remains available
during a read. Status reports successful on/off transitions, while read results
record whether blinking was retained or interrupted.

The installed Build 44 read 48 MiB across 16 KiB, 4 KiB and 512-byte blocks
with white blinking active throughout. All hashes matched the Build 43
lights-off reference, with zero new SD errors. Settings and root files were
preserved; cancellation and stopping lights during a read were also checked.
Evidence: `build/diagnostics/sd-white-blink-20260921-155412/`.

Under **LED wiring setup**, stop the rotor and choose **Light arms in order**.
Arm 1 lights red, then arms 2, 3, and 4 light white one at a time, with a short
dark gap between arms. From your viewing position, select whether **The lights
moved** clockwise or counterclockwise, and separately set **The rotor normally
spins** using that same viewing position. Choose **Save arm wiring**. Both
directions persist across restarts and determine the arm offsets for playback
and spatial tests. The rotation setting describes the motor's motion; it does
not control the motor. The diagram shows relative arm order with arm 1 at the top.
The identification test requires at least three active arms, stops after
60 seconds, and lights whole arms uniformly at no more than 10% brightness.
Saved brightness and center fade settings are retained.

On **LEDs**, **Fade from center to tips** adds a linear brightness ramp along
every arm. Set **Center brightness (%)** for the hub; the existing regular
brightness controls the tips. For example, center 10% and regular 50% fades
from 10% at the hub to 50% at the tip. The center is limited to regular
brightness, and regular 0% turns everything off. The same saved profile applies
to playback and LED patterns through both GPIO and DMA; the DMA benchmark's
10% cap scales the whole profile down. Disabling the fade restores uniform
brightness. Existing installations initially keep the fade disabled.

If the strips remain dark, use **Wiring and signal check** on that page.
Select shared clock **GPIO42**, apply **Hold LOW**, then **Hold HIGH**, and
measure at the ESP header against common ground (approximately 0 V / 3.3 V).
Follow the change through the buffer to CI at the strip input, then repeat
for the active DI pins **21, 38, 18, 7**. If a buffer input changes but its
output does not, check that chip's supply, enable, and direction. Confirm
the strip's DI/CI input end. **Toggle at 1 Hz** is also available. This is
a static/slow voltage check, not LED data; stop it before retrying solid red.
Flashes caused by touching wires establish that LEDs receive power, but do
not establish a working clock/data path. GPIO readback and transfer counters
also do not prove that signals reach the strips. **GPIO1 is assigned to motor speed PWM**
and excluded from LED output and signal checks. The previous GPIO1
clock assignment was incorrect; the designer confirmed **GPIO42**.

NVS continues to use the existing `display` namespace and key types. Missing
settings can be restored from `/config/settings.ini` on SD. The initial board
migration selects four arms; the native build fixes Hall sensing at one
falling-edge pulse per revolution. Fewer active arms use the beginning of the
same connector list and retain the physical 90-degree spacing.

Wi-Fi starts in AP-only mode when no router SSID is saved. With a saved router,
automatic connection attempts pause while a local client uses POV-Spinner,
and stop after three failed attempts. **Retry router connection** permits one
attempt while using the AP; saving router settings also requests a connection.
Connected router links stay up. An explicit router connection can change the
AP's channel because both interfaces share one radio.

Both Wi-Fi interfaces use 20 MHz channel width. Build 28 restricts the setup
AP and router connection to HT20 after USB logs captured repeated security
association timeouts (reason 209) with this PC's adapter in HT40 mode.
WPA2 encryption and Protected Management Frames remain enabled; the change
does not alter credentials. `/diag/wifi` reports `apBandwidthMHz`.

For connection trouble, **Wi-Fi details** (`/diag/wifi`) reports AP client
disconnects/reasons, router attempts, channel, uptime and reset reason. The
Wi-Fi card also shows the firmware build time. Logs record AP joins/leaves
and router failures. A resetting uptime indicates a board restart; a reported
brownout reset points to the supply. These diagnostics help distinguish that
from radio/client disconnections. The radio changes require validation on the
actual board; they do not establish the cause of the reported dropouts.

A saved Build 18 crash identified a `sys_evt` stack overflow. Wi-Fi callbacks
now only record events; the network worker formats the connection logs. AP
client counts come from the serialized join/leave events instead of querying
the driver from its callback. The event-task stack is increased to 4096 bytes,
and `/diag/wifi` reports `eventTaskMinFreeStack_bytes` for checking headroom.
Build 19 was installed over USB and passed 12 quarter/alternating test switches,
three AP disconnect/reconnect cycles, and a network scan without restarting.
The minimum reported event-task headroom was 2428 bytes, saved settings were
unchanged, and the earlier crash dump was preserved. This was a stationary
check (0 RPM); rotating output still needs validation. The SD card timed out
at mount during this check, so SD playback was not exercised.

FSEQ playback supports v1/v2 uncompressed and v2 zlib with sparse ranges and
multiple frames per compressed block. Zstd is rejected with an export hint.
Frames and compressed/expanded blocks are each limited to 4 MB and must also
fit available memory. Frame storage prefers PSRAM; LED staging uses internal
RAM. File transfers stop playback and suppress automatic playback until done.

The application uses a playback task on Core 0, a display task on Core 1,
native GPIO interrupts, and `esp_timer` notifications. Normal playback uses
parallel LCD_CAM/GDMA at 20 MHz. Frame loading and decompression use a separate
buffer outside the display lock; a 100 ms playback load timeout blanks the
LEDs and stops the sequence. The **LEDs** page includes a DMA benchmark at
4, 8, or 16 MHz, comparing GPIO and DMA with the same LED framing. It restores
GPIO afterward. See [`DMA_TEST.md`](DMA_TEST.md) for installation, test steps,
timing definitions, and oscilloscope checks. `/diag/spi` reports the current
output statistics; `/diag/dma` retains the benchmark results until reboot or
the next run. These timings exclude SD/frame preparation and are not an
operating-RPM guarantee. See [`ESP_IDF_PORT.md`](ESP_IDF_PORT.md)
for migration details and [`PCB_PIN_REVIEW.md`](PCB_PIN_REVIEW.md) for wiring.

## Validation

The native ESP-IDF application and bootloader compile and link locally.
Host tests cover FSEQ bounds, compression indexes, sparse channel mapping,
and LED framing. Browser tests exercise settings, file uploads, diagnostics,
error handling, and desktop/mobile layout against a mock HTTP backend.
Ver1.2 Build 6 was installed over OTA and tested with the four-arm sequence.
The user confirmed consistent spoke widths with DMA playback. Controlled
slow-SD and corrupt-frame tests stopped playback and allowed recovery without
restarting the controller. See [`DMA_TEST.md`](DMA_TEST.md) for measured timings
and the test limits, and [`FOUR_SPOKES_SETUP.md`](FOUR_SPOKES_SETUP.md) for the
verified image mapping.

With Visual Studio C++ build tools available:

```powershell
.\tests\run-native-tests.ps1
```

Optional browser checks use Node.js and the installed Microsoft Edge:

```powershell
npm install --prefix build/ui-test --no-audit --no-fund playwright@1.51.1
node tests/web_smoke.cjs
node tests/wifi_setup.cjs
node tests/led_pages.cjs
```
