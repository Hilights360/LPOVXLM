# SD 4-bit investigation — September 18, 2026

## September 21: rotating sequence skip measurement, Build 44

At the user's request, started `/Test4_160_CW_POV.fseq` with the existing
160-spoke mapping, 40 FPS, four 144-pixel arms, 3%-to-30% center fade and
4-bit/20 MHz SD mount. This is a 1,200-frame, 69,120-channel zlib sequence.
Saved settings were preserved; white blinking was replaced by sequence playback.

The measured 60-second retry delivered **1,916 animation frames, averaging
31.93 FPS against a 40 FPS target**. The observed sequence index advanced
2,400 frames across two loops, indicating approximately **484 skipped animation
frames (20.2%)**. Frame index and publication counter come from separate HTTP
snapshots, so the skipped count is approximate. Maximum completed frame load was
**62.316 ms**, compared with the **25 ms** frame budget; this timer includes
file access and decompression. There were **zero new SD I/O errors and zero
new 100 ms playback stalls** in this measured minute.

The Hall-derived RPM median was 102.354, with sampled range
101.027-34,522.441 RPM and 108 accepted pulses over the minute. Two of 61 status
samples exceeded 200 RPM. These readings are not independently measured physical
speed. The raw missed-spoke counter increased by 147,279; its calculation uses
the current Hall period, so short-period spikes can grossly inflate that count.
It cannot be treated as a literal count or percentage of visible missing spokes.
There were also 260 increments of the per-arm late-paint-attempt counter and
3.811 ms maximum late blanking start. No direct optical measurement was made.

An initial attempt stopped after publishing three frames with a 100 ms
frame-load timeout while the test fetched `/fseq/header`, a large 1,200-block
response. The route sends that response while holding the application lock;
this is a potential observer effect, not proof of an SD bus fault. Automatic
recovery remounted at 1-bit/20 MHz. Retrying the preferred mount restored
4-bit/20 MHz without changing saved settings. The measured retry omitted the
header transfer, polled status once per second, and fetched timing counters
only at the start and end. Playback continued after the measured minute, then
**failed about 82.4 seconds after the first playback status sample** with a
confirmed **ESP_ERR_TIMEOUT on CMD17, a 512-byte read at 4-bit/20 MHz**. This
occurred after the observation loop ended, without another FSEQ-header request.
The firmware stopped the lights on the 100 ms frame-load threshold and recovered
at **1-bit/20 MHz**. Final state is **playback stopped**, one new SD transaction
error, and two total frame-load stops including the initial diagnostic-heavy
attempt. The failing in-flight read is not included in the 62.316 ms maximum
completed-load metric. See `after-report-status.json`, `after-report-timing.json`
and `after-report-logs.txt` for the confirmed later failure.

Measured evidence: `build/diagnostics/rotating-sequence-20260921-160154/`, including `samples.jsonl`,
`summary.json`, `assessment.json`, and final timing/status snapshots. Initial
attempt: `build/diagnostics/rotating-sequence-20260921-160101/`.

## September 21: rotating white-blink read comparison, Build 44

The user started rotation and requested the same SD reads again. Testing waited
for positive RPM and advancing Hall counts over three consecutive polls.
White blinking remained active, with the same 500 ms on/off cycle, 3%-to-30%
center fade, 4-bit/20 MHz SD mount, phase 0, gated clock and active Wi-Fi.
No firmware or settings changes were made.

All three 16 MiB reads passed, totaling **48 MiB during rotation**, with the
same SHA-256 as both stationary comparisons:
`a903911d725683bf6cc77e1d906609d21a62b4ec7968d9b785d496466f3ef712`.

| Read block | Read I/O rate | Elapsed | LED transitions during job |
| --- | --- | --- | --- |
| 16 KiB | 5.885 MiB/s | 3.286 s | 7 |
| 4 KiB | 5.730 MiB/s | 3.367 s | 7 |
| 512 bytes | 1.944 MiB/s | 9.705 s | 19 |

**Zero new SD errors** occurred. Every job retained blinking, both LED phases
were observed while reads were active, and Hall counts advanced during every
job. Files and settings were preserved. White blinking remains running.

Rotation was not a perfectly stable measurement: the Hall-derived RPM readings
were mostly near 106, with reported range 105.217-1549.507 RPM and 39 accepted
pulses over the 17.672-second test series. Six of 45 status samples exceeded
200 RPM, including repeated observations of the same last Hall interval.
These are sensor-derived readings; physical speed changes or spurious pulses
were not independently established. The spikes did not coincide with any
detected SD corruption. No SD write test was run under this rotating load.

Evidence: `build/diagnostics/sd-rotating-blink-20260921-155717/`, including
per-read status observations, digests, the summary and `rotation-quality.json`.

## September 21: white blinking during SD reads, Build 44

The user requested a white blinking pattern while rereading the stationary
unit's SD card. Existing SD jobs stopped all LED modes, so Build 44 adds a
white blink diagnostic and an explicit read-only `keepBlink=1` option that
preserves it. The display task supplies the pattern independently of SD data;
the read worker retains exclusive SD ownership. Ordinary SD jobs still stop
the LEDs. No saved brightness, wiring, Wi-Fi or SD settings were changed.

The installed pattern uses all four 144-pixel arms, white for 500 ms and black
for 500 ms, with the saved center fade from 3% at the hub to 30% at the tips.
The unit remained at 0 RPM, SD at 4-bit/20 MHz, Wi-Fi active, phase 0 and gated
clock. Three 16 MiB reads of `/test2.fseq` completed with matching SHA-256
`a903911d725683bf6cc77e1d906609d21a62b4ec7968d9b785d496466f3ef712`, identical
to the prior lights-off run:

| Read block | Read I/O rate | Elapsed | LED on/off transitions during job |
| --- | --- | --- | --- |
| 16 KiB | 5.758 MiB/s | 3.350 s | 7 |
| 4 KiB | 5.644 MiB/s | 3.458 s | 7 |
| 512 bytes | 1.934 MiB/s | 9.718 s | 19 |

Status polling observed both illuminated and dark phases while each job was
actively reading; no job reported a blink interruption. Transition counters
increment only after successful LED output transfers. They confirm firmware
activity, not an external measurement of LED current or optical output.
All 48 MiB passed, with zero SD transaction errors and unchanged root file
metadata/settings. No SD write benchmark was run under blinking load.

Control checks confirmed that requesting retained blinking while stopped is
rejected, cancelling a read leaves the requested blink running, and Stop lights
works while a read is active. That last control test completed its 16 MiB read
with the same hash and correctly marked the blink condition as interrupted;
it is excluded from the 48 MiB continuous-blink result above.

The firmware build, LED-page tests, existing web smoke checks and installed
desktop/mobile control/status rendering passed. White blinking remains running
at the user's saved brightness. The scope help text now explicitly notes that
CLK has no pull-up. Evidence: `build/diagnostics/sd-white-blink-20260921-155412/`;
installation: `build/diagnostics/sd-scope-install-20260921-155314/`.

## September 21: stationary retest on Build 43

At the user's request, the stationary unit was tested at its existing
**4-bit/20 MHz** mount, with Wi-Fi AP+STA active, LED playback stopped,
normal input phase 0 and gated clock. RPM remained zero and the Hall pulse
count did not change throughout. No firmware or saved settings were changed.

All tests passed in 79.25 seconds:

- Three 16 MiB reads from `/test2.fseq`, using 16 KiB, 4 KiB and 512-byte
  blocks, produced the same SHA-256:
  `a903911d725683bf6cc77e1d906609d21a62b4ec7968d9b785d496466f3ef712`.
  These establish repeatability of that 16 MiB prefix across block sizes.
- Five consecutive 16 MiB temporary-file write/read tests verified every
  byte: **80 MiB written and 80 MiB read back**, with all temporary files
  removed. Write I/O rates were 3.289-3.327 MiB/s; readback I/O rates were
  5.969-6.070 MiB/s. The longest write operation was 56.746 ms.
- Full downloads of the 2,038,478-byte `/4_Spokes_80_POV.fseq` before and
  after testing both matched its existing reference SHA-256
  `fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81`.
- **Zero new SD transaction errors**, no recovery or profile reduction,
  unchanged root file metadata and saved settings, and no controller restart.

The controller remains mounted at 4-bit/20 MHz with the original automatic
fallback/recovery preferences. The final verified speed test is retained on
the SD Card page. This is a successful stationary run with playback stopped;
it does not isolate rotation, electrical load or power-path effects in the
earlier intermittent failures. The prior scope capture was archived before
starting SD tools. Evidence: `build/diagnostics/sd-stationary-20260921-115400/`.

## Initialization failure and later recovery in the unit

Latest retry at 17:49 on September 18 **mounted successfully at 1-bit/20 MHz**.
DAT0 now read high before initialization, along with CMD and DAT1-3. All 16
root directory entries were readable. A complete 2,038,478-byte download of
`/4_Spokes_80_POV.fseq` matched the previously recorded SHA-256
`fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81`, with
**zero new SD errors** across mounting, listing, and download. The saved
preference remains 1-bit/20 MHz with 400 kHz fallback and error recovery enabled.
The user has not reported what changed immediately before this successful retry;
the cause of the earlier low DAT0 state is still unconfirmed. This retry did
not include a write benchmark. Evidence:
`build/diagnostics/sd-retry-20260918-174952/`.

Earlier, at the user's request, **1-bit/20 MHz became the saved preference**,
with fallback to 1-bit/400 kHz and error recovery enabled. The controller had
restarted and initially reported a 4-bit mount, but CMD17 reads failed. The
requested 1-bit mount completed in 355 ms; subsequent directory reads also
failed with `ESP_FAIL` on CMD17/512-byte transfers. The empty directory response
does not establish that the card's files are gone. Trying the 1-bit/400 kHz
backup also failed, leaving the card unmounted. DAT0 still read low before
initialization. No formatting was performed. Evidence:
`build/diagnostics/sd-onebit-request-20260918-174415/`.

After the successful 20 MHz tests below, the user installed the controller on
the unit's power supply. They report that the card remained seated and no
holder, latch-ground, or capacitor changes were made. Before the LED wiring
update, the Build 32 status already showed mounting failures; Build 33 has the
same failure. The LED update preserved the SD settings.

A fresh mount retry at 17:29 on September 18 failed after 30.513 seconds:
4-bit/20 MHz, 1-bit/20 MHz, and 1-bit/400 kHz all returned `ESP_ERR_TIMEOUT`.
Before every attempt, with CMD and all DAT pins configured as inputs with
pull-ups, the firmware read `CMD=1 D0=0 D1=1 D2=1 D3=1`. DAT0 had read high
during the successful tests. The new evidence points to DAT0 being held low
or a card power/state/connection problem, but does not identify the component
or prove the unit's supply is faulty. The current web logs do not identify
the exact failing initialization command; USB serial is unavailable.

The card is currently unmounted. No format or file-write test was attempted.
A full power removal with the card left seated was requested as the next
comparison. The existing preferred 20 MHz setting, backup, and recovery flags
remain unchanged. Evidence:
`build/diagnostics/sd-mount-recheck-20260918-172946/` and the pre-update snapshot
`build/diagnostics/led-wiring-build33-20260918-172544/before.json`.

## Earlier verified 20 MHz result

**4-bit/20 MHz passed 32 consecutive 16 MiB write/read tests on
one mount**, totaling 512 MiB written and 512 MiB read/verified over 347.281
seconds, with zero new SD errors. The divider-dependent output timing described
below is now the leading explanation for the intermediate-clock failures.
**Build 32 is now installed**, with the user's requested saved default of
4-bit/20 MHz and automatic backup to 1-bit/20 MHz, then 1-bit/400 kHz.
The setting survived a reboot. The primary and backup profiles passed the
post-installation tests described below.

The fault was reproduced on two cards. After the user reported good continuity,
Build 30 passed 120 MiB of hashed read-only tests, but write/read benchmarks still
failed in 4-bit mode. Build 31 USB diagnostics identified **data CRC errors
(0x109, hardware DCRC)** during CMD25 writes, followed by cleanup response
timeouts. All four input sampling phases failed at both 4 and 10 MHz across the
comparisons; continuous clock did not prevent failure. One 4-bit/400 kHz
write/read benchmark passed. Repeated 1-bit/10 MHz benchmarks passed.
The underlying electrical/driver cause remains unconfirmed. Build 31 is
installed, and the controller was restored to its original **1-bit/10 MHz**
preference, with automatic fallback and error recovery enabled. A later trial
after the user fitted a **220 pF disc capacitor marked `221`** passed three 16 MiB tests
at 4-bit/4 MHz, but 4-bit/10 MHz still failed. Subsequent tests with reported
**100 nF** and **47 uF** additions also failed at 4-bit/10 MHz. See the capacitor
comparisons below; post-change supply waveforms remain unreported. A further
trial with **47 uF plus a soldered 220 pF capacitor** also failed at 4-bit/10 MHz.
A subsequent series, after the user reported grounding the card latch, passed
two full 16 MiB tests at 4-bit/10 MHz, then failed the third. The user clarified
that they **removed the grounding connection to type while the test was running,
and it then failed**. After instructions to secure a latch-to-PCB ground
connection, the requested repeat again failed at 4-bit/10 MHz. The actual
grounding connection has not been independently inspected or measured.
The latest requested clock comparison again failed at 10, 4, and 1 MHz in
4-bit mode, while a **4 MiB write/read test at 400 kHz passed**. The controller
remains restored to 1-bit/10 MHz, with fallback and recovery enabled.

## Divider hypothesis: 20 MHz comparison

The user's supplied analysis identified a clock-divider distinction missed in
the earlier firmware audit. On the installed ESP-IDF 5.3.3,
`sdmmc_host_get_clk_dividers()` selects host/card dividers 8/0 at 20 MHz,
2/4 at 10 MHz, 2/10 at 4 MHz, 2/40 at 1 MHz, and 10/20 at 400 kHz.
The S3 HAL initializes `phase_dout=1` (90 degrees), and the host enables
`use_hold_reg` when issuing commands. The corresponding nominal quarter-periods
of the first-stage clock are 12.5 ns at the 20 MHz profile, 3.125 ns for those
1-10 MHz profiles, and 15.625 ns at 400 kHz. Lowering the final card clock
within that custom-frequency band therefore does not increase this delay.
These are calculated internal phase delays, not measured hold times at the
socket; GPIO and board skew still affect the actual timing.

Without changing firmware or requesting a hardware change, Build 31 ran the
following sequence with fallback/recovery disabled, phase 0, gated clock,
16 KiB application blocks, and a 16 MiB target for every run:

| Order | 4-bit clock | Result | New failed SD transactions |
|---|---:|---|---:|
| 1 | 20 MHz | 16 MiB written and verified, 10.284 s | 0 |
| 2 | 10 MHz | Write failed after 208 KiB; CMD25 data CRC | 3 |
| 3 | 20 MHz | 16 MiB written and verified, 10.248 s | 0 |
| 4 | 20 MHz | 16 MiB written and verified, 10.291 s | 0 |

USB again captured the first error as `process_data_status: error 0x109
(status=00000088)` in the 10 MHz run, followed by cleanup timeouts. All three
20 MHz runs completed with no new SD transaction errors: 48 MiB written and
48 MiB read/verified in total. Writes measured about 3.28 MiB/s and reads
5.96-6.00 MiB/s. Evidence is in
`build/diagnostics/sd-divider-20mhz-20260918-164242/`, produced by
`build/run_sd_divider_20mhz.py`.

This comparison strongly supports divider-dependent output timing as the
leading explanation. It also explains why changing input sampling phase was
not an adequate test of the write path. The earlier assumption that each
lower clock would improve the relevant timing margin was incomplete. Faster
operation passing is not, by itself, proof excluding all signal-integrity or
power interactions, nor does it identify an individual DAT line.

At the time of this comparison, the automatic recovery ladder still included
10/8/4/1 MHz. Build 32 subsequently removed those profiles and adopted the
tested 20 MHz default with 1-bit backup. A 16 MHz profile would use
host divider 10 with no card divider, but has not been tested. Note that
requesting 13333 kHz in this SDK rounds the host divider up to 13, producing
about 12.307 MHz, not exactly 13.333 MHz. Drive-strength changes have not been
tested and were not needed for the three 20 MHz passes.

The test restored the original 1-bit/10 MHz preference, fallback and recovery
flags, checked the original root file metadata and settings, and removed its
temporary files. No firmware was installed during this comparison.

Primary implementation references:
[ESP-IDF 5.3.3 host divider and hold-register setup](https://github.com/espressif/esp-idf/blob/v5.3.3/components/esp_driver_sdmmc/src/sdmmc_host.c)
and [ESP32-S3 HAL clock phases](https://github.com/espressif/esp-idf/blob/v5.3.3/components/hal/esp32s3/include/hal/sdmmc_ll.h).

## 20 MHz endurance test

At the user's request, `build/run_sd_endurance_20mhz.py` ran **32 consecutive
16 MiB write/read benchmarks** at 4-bit/20 MHz on Build 31. The card was mounted
once for the entire series, with automatic fallback and error recovery disabled,
input phase 0, gated clock, and the existing 16 KiB application buffers. There
were no remounts between benchmarks. These were repeated temporary-file tests,
not a single 512 MiB file; each test flushed and verified its complete contents
and removed its file.

- All 32 tests passed: **512 MiB written plus 512 MiB read and verified**.
- Elapsed time: **347.281 seconds (5 minutes 47 seconds)** including host polling
  and gaps between tests, excluding the initial mount and final restoration.
- Zero new failed SD transactions, CRC errors, or timeouts, including restoration
  and the final directory check. USB captured all 32 completion messages and no
  SD failure, brownout, or panic messages.
- Write throughput: 3.245-3.318 MiB/s; read throughput: 5.773-5.995 MiB/s.
- Longest measured application write call: 64.473 ms; read call: 21.024 ms.
- Every completed run still reported 4-bit/20 MHz, fallback disabled, and error
  recovery disabled. The pass cannot be attributed to silent 1-bit fallback.

Evidence, including per-run status, serial capture, aggregate results, and final
checks, is in `build/diagnostics/sd-endurance-20mhz-20260918-164751/`.
The script restored the original 1-bit/10 MHz setting and original fallback and
recovery flags, verified controller settings and root file metadata against
the baseline, and left no new temporary files. A final status read confirmed
the restored card was ready and that its SD error counter had not increased.
This extends the evidence for 20 MHz stability to this five-minute workload;
longer operation, other cards, and concurrent playback remain separate cases.

## Build 32: saved 20 MHz default and 1-bit backup

The user requested making 4-bit/20 MHz the default, with 1-bit as backup.
Build 32 was built, installed over OTA, and configured accordingly:

1. 4-bit/20 MHz (preferred).
2. 1-bit/20 MHz (first backup).
3. 1-bit/400 kHz (last resort).

Mount fallback and recovery after SD I/O errors are both enabled. Recovery
continues after the failed profile, never wraps, and does not retry 4-bit
after entering 1-bit backup. Explicit 40 MHz settings remain available, but
automatic backup from a 4-bit profile never raises the 1-bit clock above
20 MHz. Retired 1/4/8/10 MHz preferences normalize to 20 MHz, preserving the
selected bus width; the controller's preferred width was explicitly saved as
4-bit for this request. The UI and API offer only 400 kHz, 20 MHz and 40 MHz.
The formatting path also uses at most 20 MHz in 1-bit mode instead of its old
10 MHz cap; no format operation was run during validation.

Validation on the installed firmware:

- Preferred 4-bit/20 MHz: 16 MiB write/read verification passed in 10.317 s.
- Manual recovery through the production recovery worker selected 1-bit/20 MHz
  on its first attempt. Its 16 MiB write/read verification passed in 20.481 s.
- A second recovery selected 1-bit/400 kHz and read the original directory.
  Another recovery request correctly returned HTTP 409 at the end of the
  ladder, rather than looping to a previously failed profile.
- Retry mount returned to 4-bit/20 MHz. A reboot retained that preference and
  both recovery flags, followed by another passing 16 MiB verification in
  10.267 s.
- The native tests, browser smoke tests and firmware build all passed. The
  recovery tests cover the direct 1-bit backup, exhaustion, strict mode,
  retired-frequency normalization, and absence of intermediate-clock profiles.

These checks exercise the backup worker manually; a CRC fault was not injected
into the now-passing 20 MHz configuration. Final state is **4-bit/20 MHz**,
fallback enabled, error recovery enabled, with other controller settings and
original root file metadata preserved and temporary benchmark files removed.
Evidence is in `build/diagnostics/sd-default-backup-build32-20260918-165859/`;
the installation/verification harness is `build/install_sd_default_backup.py`.

## PCB review

The latest close-up supplied during this investigation matches `BoardPins.h`:

| Signal | ESP GPIO | MEM2067 contact |
|---|---:|---:|
| CLK | 11 | P5 |
| CMD | 12 | P3 |
| DAT0 | 10 | P7 |
| DAT1 | 9 | P8 |
| DAT2 | 14 | P1 |
| DAT3 | 13 | P2 |
| 3.3 V | — | P4 |
| GND | — | P6 |

The socket's CLK/VDD order matches the
[GCT drawing](https://gct.co/files/drawings/mem2067.pdf). The earlier suspected
swap is withdrawn for this image. The designer confirms external 10 kΩ
pull-ups on CMD and all four DAT lines, consistent with
[Espressif's requirements](https://docs.espressif.com/projects/esp-idf/en/v5.3.3/esp32s3/api-reference/peripherals/sd_pullup_requirements.html).
The user subsequently reported that assembled-board continuity checks passed.
Continuity does not establish signal quality during a transfer. Firmware uses
ESP-IDF's documented slot-width and GPIO setup;
the SD pins do not overlap the configured LED outputs.

## Original 8 GB card: live results

The rotor was stationary, playback and lighting tests were idle, and automatic
playback/background playback were disabled. The comparison downloaded the
existing `/4_Spokes_80_POV.fseq`, **2,038,478 bytes**, and checked its SHA-256
against the 1-bit reference and the previously verified local sequence:

```text
fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81
```

Automatic fallback and error recovery were disabled during each selected test
profile, so a successful download could not silently fall back to 1-bit.
Passing downloads had the complete byte count and matching hash.

| Bus width | Clock | Complete matching reads | SD transfer failures |
|---|---:|---:|---:|
| 1-bit | 10 MHz | 3 | 0 |
| 1-bit | 4 MHz | 2 | 0 |
| 4-bit | 10 MHz | 2 | 0 |
| 4-bit | 4 MHz | 2 | 1 |
| 4-bit | 400 kHz | 2 | 0 |

The failed 4 MHz attempt produced **ESP_ERR_TIMEOUT (0x107) on CMD17**, a
512-byte single-sector read, at controller uptime 454,334 ms. The retained
SD error count increased from 3 to 4. The HTTP download was then truncated.
The harness recorded 16,384 complete bytes plus an `IncompleteRead` exception
containing another 13,312 bytes; neither was a complete file. Two subsequent
4-bit/4 MHz reads passed after remounting, with no new SD errors.

An initial download of the larger `/4_Spokes.fseq` in 1-bit mode was stopped
by the host's 90-second download limit after 15,171,584 bytes. It added no SD
transfer error and is excluded from the table. Wi-Fi download time is not an
SD throughput measurement.

Earlier Build 29 evidence also records a **4-bit/10 MHz write failure** after
1,753,088 bytes of a 4 MiB speed test, before the read phase began. Its last
reported command was CMD24, but close/cleanup also failed, so that retained
last error cannot identify the original failing write command. A later
1-bit/10 MHz 4 MiB write/read verification passed. No write speed tests,
formatting, firmware uploads, or reboots were invoked during this initial pass.

USB capture was initially unavailable because an existing ESP-IDF monitor held COM30.
The results above use HTTP status, error counters, logs, and verified downloads.
The error API does not distinguish command-response timeout from data timeout.

## Replacement 64 GB card

The user installed another card. A fresh mount reported **62,534,975,488 bytes**,
versus **7,990,673,408 bytes** for the original card. Build 29 and the controller
settings were unchanged, and the board remained stationary with playback idle.

The card contained existing speed-test files. The read comparison used
`/.lpov-speed-9afb30c7.tmp` (1 MiB), whose reference SHA-256 was:

```text
5855a2a0ff6e1ab5c173009480c3d69568fd632b1c0b1cae90b0e6713d156607
```

One 1-bit/10 MHz reference download and two downloads at each of 4-bit/10 MHz,
4-bit/4 MHz, and 4-bit/400 kHz passed with matching hashes and no new SD errors.
These downloads are paced by HTTP and use a different buffering path from the
16 KiB-block SD benchmark; their success does not validate sustained transfers.

The existing firmware's 4 MiB temporary-file benchmark was then run twice at
each setting, with automatic fallback and recovery disabled:

| Setting | First test | Repeat with USB capture |
|---|---|---|
| 1-bit / 10 MHz | 4 MiB written/read/verified | 4 MiB written/read/verified |
| 4-bit / 4 MHz | Read failed after 80 KiB | Read failed after 208 KiB |
| 4-bit / 10 MHz | Read failed after 32 KiB | Read failed before verifying the first block |

Each 4-bit run completed its 4 MiB write/flush phase, but failed during reading;
the written file was therefore not fully verified. Both 1-bit runs completed
without new bus errors, at approximately 1.07 MiB/s write and 1.14 MiB/s read.

After the user closed the existing serial monitor, COM30 capture showed this
sequence at **both 4-bit clocks**:

```text
sdmmc_read_sectors_dma: sdmmc_send_cmd returned 0x109
diskio_sdmmc: sdmmc_read_blocks failed (0x109)
... cleanup follows ...
sdmmc_read_sectors_dma: sdmmc_send_cmd returned 0x107
diskio_sdmmc: sdmmc_read_blocks failed (0x107)
```

ESP-IDF 5.3.3 defines `0x109` as `ESP_ERR_INVALID_CRC` and `0x107` as
`ESP_ERR_TIMEOUT`. The status page retained the later CMD17 timeout, hiding the
earlier CRC failure. The serial log establishes the first error, but its INFO
logging does not identify the original command number or distinguish a response
CRC from a data CRC. The bus-error count increased from 4 to 12 across the four
failed benchmarks, with two errors per run. No reboot or firmware change was
needed to capture them.

DAT0 and DAT1 briefly read LOW during the first remount immediately after the
10 MHz failure, then all lines read HIGH on the following mount. Because the
card had just suffered an interrupted transfer, this observation does not by
itself establish a short or a missing pull-up.

The newly created benchmark files were removed after remounting at 1-bit.
All pre-existing files, including the four older temporary files, were retained;
root file metadata and controller settings matched their pre-test snapshots.
The reference file also retained its original hash after the first benchmark
series. The board was left mounted at **1-bit/10 MHz**, fallback and recovery on.

This rules out the original card being the sole explanation. It supports
investigating the shared 4-bit signal path and sustained-transfer timing; it
does not yet distinguish a board electrical fault from a driver/timing problem.

## After continuity checks: Build 30

After power was reconnected, the controller reported the original **8 GB card**
(7,990,673,408 bytes), rather than the preceding 64 GB card. Build 30 added an
exclusive read-only worker, SHA-256, temporary sampling/clock controls, and an
eight-entry SD transaction error history. The history preserves the first CRC
failure instead of exposing only the subsequent cleanup timeout.

Two 15-profile matrices each read 4 MiB per profile:

- The prefix of the existing `/4_Spokes.fseq`, checked against an earlier host
  reference (`1c81d621b5c01f6dd3f697e45705c4a04332a4b2c8552978b00a444179869b69`).
- A deterministic random file uploaded in stable 1-bit mode, checked against
  its host reference (`49ead74c664847c929c262c4a7c39a7a263707b6464b793c9b5e346c7ec374a1`).
  Only this newly created file and its possible upload staging file were removed.

Each matrix included 1-bit/10 MHz; 4-bit/4 MHz at 512-, 4096-, and 16384-byte
read blocks; 4-bit/10 MHz at 512- and 16384-byte blocks; all four input delay
phases at 4 and 10 MHz; continuous-clock operation at both clocks; and
4-bit/400 kHz with normal timing. **All 30 reads passed**, totaling **120 MiB**,
with matching hashes and no new bus errors. The read-only operation reports a
digest, not `verified=true`; the host comparison established matching content.
Normal 4-bit/10 MHz reads measured about 3.4 MiB/s of file I/O time.

The unchanged write/read benchmark still failed between the two read matrices:

| Setting | Build 30, 4 MiB write/read benchmark |
|---|---|
| 1-bit / 10 MHz | Complete and verified, no new bus errors |
| 4-bit / 4 MHz | Write failed after 2,916,352 bytes |
| 4-bit / 10 MHz | Write failed after 1,179,648 bytes |

Both original errors were **ESP_ERR_INVALID_CRC on CMD25, 4096 bytes**; later
CMD24/512-byte cleanup writes timed out. Neither failing test reached the read
phase. Failed test files were removed after remounting in 1-bit mode. Root file
metadata and controller settings were preserved, and 1-bit/10 MHz was restored.

This expands the fault to writes as well as reads. It also shows that sustained
read-only traffic and random data can pass. The results do not establish an
input timing correction: every tested input phase also passed at the default
phase. A failure during CMD25 alone does not distinguish data CRC from response
CRC; detailed driver logging is needed for that distinction.

## Build 31: isolate the failing write path

Build 31 adds the same temporary input-phase and continuous-clock controls to
the write/read benchmark, and enables detailed driver failure logs during it.
No GPIO mapping, drive-strength, or timeout changes were made. The same 8 GB
card remained installed. Automatic fallback/recovery were disabled for each
selected test profile and restored after cleanup.

| Setting | First 4 MiB write/read test | Follow-up |
|---|---|---|
| 1-bit / 10 MHz, normal timing | Passed | Final 4 MiB repeat passed |
| 4-bit / 4 MHz, phase 0 | CRC during write at 884,736 bytes | 4 MiB test: failed at 1,032,192 bytes |
| 4-bit / 4 MHz, phase 0, continuous clock | CRC during write at 704,512 bytes | Not repeated |
| 4-bit / 4 MHz, phase 1 | Passed | 16 MiB test: CRC during write at 1,671,168 bytes |
| 4-bit / 4 MHz, phase 2 | CRC during write at 114,688 bytes | Not repeated |
| 4-bit / 4 MHz, phase 3 | Passed | 16 MiB test: CRC during write at 212,992 bytes |
| 4-bit / 10 MHz, phase 0 | CRC during write at 966,656 bytes | Not repeated |
| 4-bit / 10 MHz, phase 0, continuous clock | CRC during write at 131,072 bytes | Not repeated |
| 4-bit / 10 MHz, phase 1 | Not in initial matrix | 16 MiB test: CRC during write at 294,912 bytes |
| 4-bit / 10 MHz, phase 2 | Not in initial matrix | 16 MiB test: CRC during write at 704,512 bytes |
| 4-bit / 10 MHz, phase 3 | CRC during write at 212,992 bytes | Not repeated |
| 4-bit / 400 kHz, phase 0 | Passed (48.4 seconds) | Not repeated |

Every failed case began with CMD25/4096-byte `ESP_ERR_INVALID_CRC`. The USB
driver log identifies the original error as:

```text
sdmmc_req: process_data_status: error 0x109 (status=00000088)
```

In the installed ESP-IDF 5.3.3 sources, `sdmmc_reg.h` defines bit 7 as DCRC and
bit 3 as DATA_OVER; bit 6 (response CRC) is absent. `sdmmc_transaction.c`
maps DCRC to `ESP_ERR_INVALID_CRC`. Later cleanup calls log
`process_command_response: error 0x107`, generally with `status=00000104`.
This distinguishes the original **data-path CRC failure** from subsequent
command-response timeouts. During a write, it does not by itself establish
whether payload signaling or the received data-status token is at fault, nor
which physical DAT line is responsible.

The two initial phase-1/phase-3 passes were not repeatable with larger requested
files; both repeats failed well before 4 MiB. Neither changing input sampling
nor keeping the clock running is a demonstrated correction. The 400 kHz pass
is one observation, not a reliability guarantee, and is slower than tested
1-bit/10 MHz operation. The next measurement is socket 3.3 V and CLK during a
failing test, followed by DAT0-3 as needed. The user has an oscilloscope.

All newly created benchmark files were removed in stable 1-bit mode. Root file
metadata and controller settings match their starting snapshots. The final
4 MiB 1-bit benchmark wrote/read/verified every byte without a new bus error.
Native tests, browser smoke tests, and the ESP-IDF build passed before OTA;
installation retained controller and SD preferences.

## First scope observation run

The user connected their single probe to C2's 3.3 V side, with ground at C2 GND,
and reported readiness. The first attempt lost Wi-Fi after mounting 4-bit/4 MHz
but before starting the benchmark. USB showed Wi-Fi disconnection events rather
than an SD transfer error. After reconnecting, one normal-timing **4 MiB
4-bit/4 MHz write/read benchmark passed**, verified every byte, and completed
in 6.498 seconds. The SD error count stayed at 34. This is an intermittent
pass; a causal effect from attaching the probe has not been established.

Wi-Fi dropped again while polling, but USB captured completion. Reconnection
allowed saving the completed result and restoring **1-bit/10 MHz**, automatic
fallback, and error recovery. Root file metadata and controller settings
matched the pre-test snapshots; the benchmark removed its temporary file.
The user reported **3.28 V** at C2. No minimum voltage, ripple amplitude, or
waveform during the test has yet been reported; the DC reading alone does not
establish transient supply stability.

Evidence: `build/diagnostics/sd-scope-check-20260918-130737/` records the aborted
pre-test attempt; `build/diagnostics/sd-scope-check-20260918-130857/` contains
the USB completion log, recovered result, restored state, and file listing.

## Scope ripple run: AC coupling

After the user confirmed readiness with the probe still at C2 and the requested
AC-coupled 50 mV/div, 1 ms/div setup, a **16 MiB** write/read test was requested
at **4-bit/4 MHz**, normal phase and gated clock. It failed during writing after
**458,752 bytes (448 KiB)**, in **465 ms**, before reaching the read phase.
USB again captured `process_data_status: error 0x109 (status=00000088)` first,
followed by cleanup command-response timeouts. The error count rose from 34 to
37. The attached probe therefore did not prevent recurrence of the SD fault.

The temporary file was removed after remounting in 1-bit mode. Original
1-bit/10 MHz settings, fallback/recovery, controller settings, and root file
metadata were restored and checked. Evidence is in
`build/diagnostics/sd-scope-ripple-20260918-131506/`. The ripple waveform and
amplitude during this reproduced failure were initially described as small.
The user subsequently reported occasional **0.5 V noise** and proposed adding
supply capacitance. Polarity, duration, peak-versus-peak-to-peak amplitude, and
correlation with the CRC error remain unspecified. Probe/channel attenuation
and a short ground connection should be checked before treating this as a
confirmed rail transient.

A proposed temporary comparison is **47 uF, at least 6.3 V electrolytic** across
C2's 3.3 V/GND pads, with short leads, correct polarity, and the existing C2
retained. Disconnect all power including USB while fitting it. This is a bench
experiment, not a confirmed permanent regulator/output-capacitor specification;
the regulator model has not been identified. Repeat the same 16 MiB, 4-bit/
4 MHz test several times after installation, comparing both noise and CRC
failures. Later capacitor trials are recorded below.

## 220 pF disc capacitor comparison

The user confirmed adding a disc capacitor and subsequently corrected the
initially reported `102` marking to **`221`, 220 pF (0.00022 uF)**. The component
was unchanged during the reported tests; this corrects its identification.
At this stage, the proposed **47 uF** bulk capacitor had not yet been tested. See the
[manufacturer capacitance-code table](https://datasheets.kyocera-avx.com/100C.pdf).
Its exact installed location, rated voltage, and post-installation ripple
amplitude have not been explicitly confirmed.

Build 31, the same 8 GB card, normal phase 0, and gated clock were retained.
Each test disabled automatic fallback/recovery and checked the actual bus mode.

| Setting | Result with the reported disc capacitor installed |
|---|---|
| 4-bit / 4 MHz | **Three consecutive 16 MiB writes/reads fully verified**, zero new bus errors; about 24.9 seconds each |
| 4-bit / 10 MHz | **Write failed at 8,945,664 bytes (8.53 MiB)** in 4.258 seconds; did not reach the read phase |

The 10 MHz run again began with CMD25/4096-byte data CRC failure; USB logged
`process_data_status: error 0x109 (status=00000088)`. Cleanup response timeouts
followed. This is an observed improvement at 4 MHz, but does not establish
the capacitor as the cause or show that 10 MHz is reliable. The subsequent
bulk-capacitance comparison is recorded below.

Both runs restored 1-bit/10 MHz with fallback/recovery enabled, removed only
their new temporary files, and verified unchanged root file metadata and
controller settings. Evidence: `build/diagnostics/sd-scope-repeat-20260918-132751/`
and `build/diagnostics/sd-cap-10mhz-20260918-132922/`, including USB logs.

## Holder pressure comparison

At the user's request, the same **16 MiB, 4-bit/10 MHz** benchmark was attempted
with pressure on the card holder. The user subsequently clarified that **no
extra capacitor was installed: this was a pressure-only test**. This supersedes
the earlier instruction/assumption to retain the preceding capacitor arrangement.
The first run failed during writing
after **540,672 bytes (528 KiB)**, in **392 ms**, before the read phase. The
repeat sequence stopped at that failure. The SD error count increased by three,
with the initial data CRC error followed by cleanup timeouts.

Pressure did not prevent failure in this trial; this observation alone does not
exclude an intermittent socket contact or solder-joint problem. The prior 220 pF
trial and this pressure-only run differ in both capacitance and pressure, so their
results do not isolate the effect of pressing the holder. Pressure location
and force were not instrumented. The user was told to release the holder after
the run. Original 1-bit/10 MHz preferences, fallback/recovery, controller settings,
and root file metadata were restored and checked, and the new temporary file
was removed. Evidence: `build/diagnostics/sd-holder-pressure-20260918-133158/`.

## 100 nF capacitor comparison (value confirmed afterward)

The user reported fitting another capacitor and requested a repeat. Its value
was initially unknown; the user subsequently identified it as **100 nF** before
requesting the next trial with 47 uF. The voltage rating and exact placement
remain unconfirmed. The user was instructed to leave the holder unpressed.

At **4-bit/10 MHz**, normal phase and gated clock, the requested 16 MiB benchmark
failed during writing after **196,608 bytes (192 KiB)**, in **211 ms**, without
reaching the read phase. USB again captured an initial data CRC failure, followed
by cleanup timeouts. This capacitor change did not prevent the reproduced fault;
it does not rule out a supply problem or identify a suitable capacitor value.

The new temporary file was removed, and 1-bit/10 MHz with fallback/recovery was
restored. Controller settings and root file metadata matched the starting
snapshots. Evidence: `build/diagnostics/sd-new-cap-10mhz-20260918-133958/`.

## 47 uF capacitor comparison

After identifying the previous addition as 100 nF, the user reported fitting
**47 uF** and requested the same test. The user was instructed to leave the
holder unpressed and observe the supply trace. Firmware Build 31, the 8 GB card,
normal input phase, and gated clock were retained; fallback and recovery were
disabled during the selected test.

The first requested **16 MiB, 4-bit/10 MHz** benchmark failed during writing
after **786,432 bytes (768 KiB)**, in **471 ms**, without reaching the read phase.
It began with CMD25/4096-byte **ESP_ERR_INVALID_CRC**; USB logged
`process_data_status: error 0x109 (status=00000088)`. Cleanup response timeouts
followed, for three new SD errors in total. The planned repetitions stopped at
that first failure.

The reported 47 uF addition did not prevent recurrence. This does not by itself
exclude supply/ground problems: exact placement and the post-change supply
waveform have not been reported or independently observed. The differing byte
offsets of these intermittent failures do not establish improvement or worsening.

The new temporary file was removed in 1-bit mode. Original 1-bit/10 MHz
preferences, fallback/recovery, controller settings, and root file metadata
were restored and verified. Evidence:
`build/diagnostics/sd-47uf-10mhz-20260918-135001/`, including USB capture.

## 47 uF plus soldered 220 pF comparison

The user reported soldering the capacitor marked **221 (220 pF)** in place and
explicitly confirmed that **both it and the 47 uF capacitor remained connected**.
The user was instructed to leave the holder unpressed. Firmware Build 31, the
8 GB card, phase 0, and gated clock were retained. Automatic fallback and error
recovery were disabled for the selected test profile.

The first requested **16 MiB, 4-bit/10 MHz** write/read test failed during writing
after **507,904 bytes (496 KiB)**, in **362 ms**, before reading. It began with
CMD25/4096-byte **ESP_ERR_INVALID_CRC**; USB again logged
`process_data_status: error 0x109 (status=00000088)`. Cleanup response timeouts
followed, producing three new SD errors. The remaining repetitions were skipped
after this failure. The combination did not prevent recurrence; the waveform
and exact connection geometry have not been independently inspected.

The failed temporary file was removed after remounting in 1-bit mode. Original
1-bit/10 MHz preferences, fallback/recovery, controller settings, and root file
metadata were restored and verified. Evidence:
`build/diagnostics/sd-47uf-221-soldered-20260918-160157/`, including USB capture.

At the user's request, the same 16 MiB test was repeated without a reported
hardware change. It again failed during writing at 4-bit/10 MHz, this time after
**2,113,536 bytes (2.016 MiB)**, in **1.120 seconds**, without reaching the read
phase. CMD25/4096-byte data CRC failure again preceded cleanup timeouts (three
new errors). The temporary file was removed and the original 1-bit/10 MHz
preferences, fallback/recovery, controller settings, and root file metadata
were restored and verified. Evidence:
`build/diagnostics/sd-47uf-221-soldered-20260918-160403/`.

## Latch-grounding report: two passes, failure after connection removal

The user requested another repeat, then reported that they had **grounded the
small latch retaining the card** since the previous failed run. They subsequently
clarified: **"i removed it to type and the test running failed"**. The third
failure must therefore not be described as a failure with the grounding
connection continuously attached. The precise original grounding method remains
unconfirmed. The last confirmed capacitor configuration
was 47 uF plus soldered 220 pF; no capacitor change was reported for this series.

All three requested 16 MiB tests used **4-bit/10 MHz**, phase 0, gated clock,
and disabled fallback/recovery:

| Run | Result |
|---|---|
| 1 | 16 MiB written/read/verified, no new bus errors; 14.155 seconds |
| 2 | 16 MiB written/read/verified, no new bus errors; 14.099 seconds |
| 3 | Write failed at **4,784,128 bytes (4.5625 MiB)**; 2.354 seconds; no read phase |

The third run began with the same CMD25/4096-byte data CRC failure, followed
by cleanup timeouts. Combined with the user's removal timing, this strengthens
the correlation between the latch connection and successful transfers. It does
not yet separate electrical grounding from pressure/contact changes or establish
a permanent repair. The next comparison should secure a short connection from
the metal latch to known PCB GND, leave the holder unpressed, and retain the
connection throughout all repeat runs. The subsequent requested trial is
recorded below.

There was also a **1-bit/10 MHz CMD17/512-byte timeout during the initial
directory listing**, before the first 4-bit test. The listing API returned an
empty array, so the script's final before/after comparison failed even though
the restored listing contained the expected files. Its restored listing was
then compared with the preceding verified session's snapshot: all **16 entries
and their metadata matched**. The test files had been removed, and original
SD preferences and controller settings were restored.

A separate **4 MiB 1-bit/10 MHz write/read recheck passed**, verified every byte,
and added no SD errors. This confirms that run's success but does not erase the
earlier 1-bit timeout. The repeat harness now rejects an empty baseline listing
or a new bus error during that listing before creating any test file.

Evidence: `build/diagnostics/sd-47uf-221-soldered-20260918-160620/` contains the
three runs, restored state, and comparison against the earlier verified listing;
`build/diagnostics/sd-onebit-recheck-20260918-160737/` contains the final passing
1-bit benchmark, unchanged file metadata/settings, and restored preferences.

## Requested repeat after latch-ground setup instructions

After being instructed to secure a short connection from the metal latch to
C2's GND pad, power up, and keep the connection attached without pressing the
holder, the user requested the test. The physical connection itself was not
independently inspected or measured. Three 16 MiB trials at 4-bit/10 MHz were
planned, with phase 0, gated clock, and fallback/recovery disabled.

The **first run failed during writing after 507,904 bytes (496 KiB)**, in
**359 ms**, before the read phase. CMD25/4096-byte `ESP_ERR_INVALID_CRC` was
the original error; USB logged data CRC status `0x00000088`. Cleanup response
timeouts followed, for three new bus errors. The remaining repetitions were
skipped, and the user was told testing had finished. The latch-grounding
observation has therefore not established a repeatable correction.

The temporary file was removed in 1-bit mode. Original 1-bit/10 MHz preferences,
fallback/recovery, controller settings, and root file metadata were restored
and verified. Evidence:
`build/diagnostics/sd-latch-ground-secured-20260918-161302/`.

## Latest requested pair and slower-clock comparison

The user requested a couple of 4-bit tests and then added slower clocks. Two
16 MiB tests were attempted at 10 MHz, with remounting between them despite
the first failure. Additional 4 MiB comparisons used 4 MHz, 1 MHz, and 400 kHz;
the smaller requested size keeps the slowest test within the worker's deadline.
No new physical change was reported. The user was instructed to keep the
hardware unchanged. All selected profiles used 4-bit mode, phase 0, gated clock,
and disabled fallback/recovery, with actual width/clock checked after mounting.

| Clock | Requested size | Result |
|---|---|---|
| 10 MHz, run 1 | 16 MiB | Write failed at **229,376 bytes (224 KiB)**; 220 ms |
| 10 MHz, run 2 | 16 MiB | Write failed at **393,216 bytes (384 KiB)**; 301 ms |
| 4 MHz | 4 MiB | Write failed at **1,392,640 bytes (1,360 KiB)**; 1.217 seconds |
| 1 MHz | 4 MiB | Write failed at **753,664 bytes (736 KiB)**; 1.986 seconds |
| 400 kHz | 4 MiB | **Fully written/read/verified**, zero new SD errors; 48.351 seconds |

Each failed run began with CMD25/4096-byte data CRC failure and then cleanup
response timeouts, adding three bus errors. None reached the read phase. USB
captured data CRC status `0x00000088`. The 400 kHz pass establishes this trial's
success, not sustained reliability; it also does not validate the higher clocks.
Its measured read rate was about 0.174 MiB/s, below earlier verified 1-bit/
10 MHz results.

After each series, the new temporary files were removed, 1-bit/10 MHz with
fallback/recovery was restored, and unchanged controller settings/root file
metadata were verified. Evidence:

- `build/diagnostics/sd-fourbit-pair-20260918-162228/`
- `build/diagnostics/sd-fourbit-slower-20260918-162259/`
- `build/diagnostics/sd-fourbit-400khz-20260918-162347/`

The user requested the complete comparison again. With the same requested
sizes and bus/timing options, and no reported hardware change, the repeat gave:

| Clock | Repeat result |
|---|---|
| 10 MHz, run 1 | Write failed before any bytes were reported written; 119 ms |
| 10 MHz, run 2 | Write failed at **65,536 bytes (64 KiB)**; 142 ms |
| 4 MHz | Write failed at **278,528 bytes (272 KiB)**; 340 ms |
| 1 MHz | Write failed at **163,840 bytes (160 KiB)**; 521 ms |
| 400 kHz | **4 MiB fully written/read/verified**, zero new errors; 48.301 seconds |

Each failure again began with CMD25/4096-byte data CRC failure and was followed
by cleanup timeouts. This is a second consecutive successful 400 kHz comparison,
while every higher-clock attempt in these two comparison rounds failed. All
new temporary files were removed; original 1-bit/10 MHz preferences,
fallback/recovery, controller settings, and root file metadata were restored
and verified. Repeat evidence:
`build/diagnostics/sd-fourbit-clock-repeat-20260918-162614/`.

## Diagnostic API

`POST /sd/read-test` accepts form parameters `path`, `mib=1|4|16`,
`block=512|4096|16384`, `phase=0|1|2|3`, and `continuous=0|1`. The latter three
default to 16384, 0, and 0. It reads the requested prefix of an existing file,
without creating or modifying that file. Playback stops and the exclusive SD
worker rejects competing storage operations. Poll `GET /sd/tools` or `/status`
and compare `sha256` with an independently known prefix hash. `verified=false`
is intentional because the firmware does not know the expected digest.

`POST /sd/speed` retains its normal 16 KiB temporary-file write/verify behavior.
Build 31 also accepts optional `phase` and `continuous` controls for comparing
the failing path. Both tests restore normal timing after completion, failure,
or cancellation; these options are not saved in NVS. `POST /sd/cancel` requests
cancellation between card operations. Both workers have a 60-second deadline
checked between operations, so a single blocked driver call can exceed it.

Sampling delay and continuous-clock controls use the documented
[ESP-IDF SDMMC host API](https://docs.espressif.com/projects/esp-idf/en/v5.3.3/esp32s3/api-reference/peripherals/sdmmc_host.html).
The project's CMake enables compiled driver failure details without modifying
the installed SDK; only the SD test worker temporarily enables `sdmmc_req`
DEBUG output. `/status` exposes the last eight transaction failures at
`sd.ioErrors.recent`, ordered by count and preserved across remounts.

## Interpretation and next bench checks

- This is an SD transaction fault, rather than just an HTTP timeout or an
  FSEQ decoding problem. CMD17 also shows it is not confined to multiblock
  transfers. The SD error alone does not identify which physical signal failed.
- Valid 4-bit transfers make a permanent DAT1–3 swap unlikely. They do not
  rule out intermittent socket contacts, solder joints, power noise, or timing.
- Slower-clock operation has not been established as a fix. The sample is
  small and contains both passing and failing reads at 4 MHz.
- A second card also failed sustained 4-bit reads. Ordinary downloads passed
  on both cards, so use the sustained benchmark when evaluating a correction.
- The user reports continuity checks passed. The pin mapping and pull-up
  values match the intended circuit; dynamic signal quality remains unmeasured.
- The fault persists across cards: scope 3.3 V at the socket and CLK/CMD/DAT
  during the failure if continuity/contact checks pass. Firmware timing remains
  a possibility. Temporary input-delay and clock-gating comparisons are described
  above; GPIO assignments, drive strength, and timeouts have not been changed.

## Evidence and preserved state

Generated evidence is in these ignored build directories:

- `build/diagnostics/sd-fourbit-20260918-123152/`: initial baseline and the
  host-limited large download.
- `build/diagnostics/sd-fourbit-20260918-123353/`: 4-bit clock comparison and
  the reproduced CMD17 failure.
- `build/diagnostics/sd-fourbit-20260918-123525/`: matching 1-bit/4 MHz tests,
  successful 4-bit/4 MHz repeat tests, and final restored state.
- `build/diagnostics/sd-stability/`: earlier Build 29 read/write evidence.
- `build/diagnostics/sd-fourbit-20260918-123904/`: replacement-card downloads.
- `build/diagnostics/sd-replacement-20260918-124011/`: replacement-card initial
  sustained benchmarks, restoration, and reference-file hash check.
- `build/diagnostics/sd-replacement-20260918-124202/`: sustained benchmark repeat
  during USB capture and restoration.
- `build/diagnostics/sd-replacement-usb-20260918-124149.log`: original CRC failures
  before the cleanup timeouts at both 4-bit clocks.
- `build/diagnostics/sd-timing-20260918-125405/`: all 15 existing-sequence
  read-only profiles, hashes, and preserved state on Build 30.
- `build/diagnostics/sd-pattern-20260918-125658/`: all 15 random-data read-only
  profiles, USB capture, reference hash, cleanup, and preserved state.
- `build/diagnostics/sd-build30-benchmark-20260918-125625/`: passing 1-bit
  benchmark and CMD25 CRC failures during 4-bit writes, with USB capture.
- `build/diagnostics/sd-write-timing-20260918-130012/`: Build 31 timing
  comparisons, including the DCRC hardware status and 400 kHz pass.
- `build/diagnostics/sd-write-timing-repeat-20260918-130156/`: larger requested
  tests disproving the initial phase-1/phase-3 passes, remaining 10 MHz phases,
  final successful 1-bit benchmark, USB capture, cleanup, and restored state.

Each run saved before/after state and compared the root directory listing.
Original SD preferences, controller settings, and root file metadata were
preserved. The retained last-error field still shows the earlier 4-bit failure;
it does not mean the restored 1-bit mount has a new error. The temporary host
harness is `build/read_sd_matrix.py`; it is specific to this board and session.


## September 21: ESP32 SD pin scope installed

At the user's request, firmware Build 39 added an ADC scope at `/sd/scope`.
Live validation found that ADC2's calibration could extrapolate saturated
inputs to roughly 4.98 V. That is not a valid measurement of the pin voltage.
Build 40 corrected this before completing installation: raw values near full
scale and converted values at/above 3,100 mV are flagged as over-range and
export no numeric voltage. The browser also rejects these values from an
older capture. When no voltage samples are in range, it displays raw counts.

**Build 40 is installed and verified.** The card stayed inserted throughout.
Each capture stopped playback, detached the SD host, sampled one analog pin,
restored all six pads to digital mode, and remounted at the original
**4-bit/20 MHz**. The saved Auto/20 MHz preference and fallback/recovery flags
were unchanged, as were all other saved controller settings.

- DAT0, DAT1, CMD, DAT2, DAT3, and CLK each returned 1,024 valid raw readings.
- Requested 100 us spacing produced average attempt rates of 4,679-5,057/s;
  the largest observed scheduling gap was 11,778 us. Samples carry timestamps.
- Every CMD/DAT reading in this validation was flagged over-range. The usable
  voltage range was insufficient to establish their HIGH-level noise amplitude.
- CLK was floating with the host detached; its trace is not an operating clock
  waveform or a measurement of noise during transfers.
- Cancellation of a 10 ms-spaced capture retained 41 readings and restored SD.
  File access correctly returned HTTP 409 while capture owned the SD bus.
- Root directory metadata matched the pre-installation snapshot. A full
  2,038,478-byte `/4_Spokes_80_POV.fseq` download matched the recorded SHA-256
  `fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81`.
- No new SD transaction errors occurred. The installed web page rendered the
  final DAT0 capture as raw counts, with 1,024 over-range flags and blank
  voltage fields in its CSV export. Desktop/mobile browser checks passed.

LED playback remains stopped after scope capture. The motor was still
reporting about 104 RPM; this firmware does not implement motor-speed control.
No formatting or SD write benchmark was run. Voltage accuracy has not been
compared against an external reference.

Evidence: `build/diagnostics/sd-scope-install-20260921-063847/`. The earlier
Build 39 captures are retained separately in
`build/diagnostics/sd-scope-install-20260921-063511/` as evidence of the
out-of-range calibration behavior; do not interpret their extrapolated
millivolt fields as measured voltages.


## September 21: Build 41 all-pin sample set

The user requested sampling all six pins and displaying them together. Build
41 is installed with an All six pins selection, a six-trace view with shared
zoom/position, a comparison table, and complete-set CSV/JSON export. The SD
host is detached once, each pin is captured sequentially, and the card is
remounted once after the set. Each trace retains its start offset; displayed
per-pin time zero does not imply simultaneous sampling.

The full set was started through the installed web page. All six channels
returned 1,024 valid raw readings, totaling 6,144. The starts were 52.419,
300.455, 553.444, 792.440, 1030.416, and 1276.411 ms into the job, in the order
DAT0, DAT1, CMD, DAT2, DAT3, CLK. All CMD/DAT voltage readings were over-range.
CLK raw readings spanned 42-1839, with converted values of 37-1575 mV under
ADC sampling. Correction confirmed September 21: the print has five pull-ups
on CMD and DAT0-3 only; CLK has no external pull-up. The scope also disables
CLK's internal pull-up and releases its output driver. Its decay is consistent
with an undriven line under ADC sampling, not evidence of a clock pull-up fault.
This capture does not observe the operating SD clock.

A cancellation check retained a completed 1,024-sample DAT0 trace and a
183-sample partial DAT1 trace. File access was blocked while the capture
owned the bus. The card remounted at its original 4-bit/20 MHz after both
cancellation and the complete set. A subsequent read after cancellation
matched the recorded 2,038,478-byte sequence SHA-256. Saved settings and root
file metadata were preserved, with zero new SD transaction errors.

The installed browser page displayed all six traces, exported all 6,144 CSV
rows, and retained the set across reload. Desktop/mobile rendering checks
passed without page errors. Voltage/raw selection, partial-set cancellation,
clipped values, and per-pin timing are covered by the browser tests. The
firmware build and existing web smoke tests passed. The full set remains in
controller RAM; LED playback is stopped.

Evidence: `build/diagnostics/sd-scope-install-20260921-065037/`, including
`all-pins-capture.json`, `all-pins.csv`, `all-pins-summary.json`, and desktop
and mobile screenshots.

## September 21: Build 42 Wi-Fi-off comparison

The user approved repeating the all-pin capture with the ESP32 radio stopped.
Build 42 adds an explicit Wi-Fi On / Off, then restore selection. The network
task stops both AP and STA, suspends scans/retries/configuration while quiet,
acknowledges the stop, and resumes the existing radio configuration afterward.
Sampling begins after a 100 ms settling delay. A 90-second limit independently
restarts Wi-Fi if capture cleanup is not reached; restart failures are retried.
The worker resumes Wi-Fi before remounting SD, including on cancellation/error.
No saved network or SD settings change. The page tolerates the expected offline
period, and capture metadata and download filenames identify the radio condition.

Two complete on/off pairs were acquired at requested 100 us spacing, with 1,024
valid raw readings per pin in every set (24,576 readings total). AP+STA was
connected before each run. The second pair's raw counts were:

| Signal | Wi-Fi on min-max | Wi-Fi off min-max |
| --- | --- | --- |
| DAT0 | 4007-4095 | 4095-4095 |
| DAT1 | 4008-4095 | 4095-4095 |
| CMD | 4059-4095 | 4095-4095 |
| DAT2 | 4030-4095 | 4095-4095 |
| DAT3 | 4049-4095 | 4095-4095 |
| CLK | 0-1337 | 75-1384 |

Both radio-off sets had all 5,120 CMD/DAT readings saturated at 4095. Dips returned
in the intervening radio-on set. This associates the visible pattern with Wi-Fi
activity, but cannot establish clean high-level voltage, quantify voltage noise,
or separate real supply/pin changes from ADC loading/reference/interference.
CLK remains variable without Wi-Fi. The designer explicitly confirmed that CLK
has no pull-up; only CMD and DAT0-3 do. With the host detached, CLK is undriven
and need not stay at 3.3 V. Its decay does not establish an SD clock fault.
SD traffic was paused for all captures, and the captures were sequential,
not simultaneous.

Each quiet set confirmed Wi-Fi stopped before the first sample and resumed after
the last sample. Quiet cancellation restored both links and the original SD
mount. SD returned at 4-bit/20 MHz, settings/root file metadata matched the
baseline, no new SD I/O errors occurred, and the reference 2,038,478-byte file
download retained SHA-256
`fbf8d228f099f38d972bd7990cdb49a334bafbf58bda457f739f35bb682f0c81`.

Validation passed: firmware build, scope browser tests including offline/reload
and unconfirmed-stop labels, Wi-Fi setup tests, existing web smoke tests, and
the installed page's six traces, 6,144-row CSV export and mobile rendering.
The second radio-off set remains in controller RAM with playback stopped.
Evidence: `build/diagnostics/sd-scope-install-20260921-071005/`, including
`wifi-on-1.json`, `wifi-off-1.json`, `wifi-on-2.json`, `wifi-off-2.json`,
`quiet-comparison.json`, `quiet-cancel.json`, `quiet-read-verification.json`,
the installed-page screenshots, and `wifi-on-off-comparison.png`/`.svg`.

## September 21: Build 43 permanent scope navigation

The user requested keeping the scope and adding a direct link. Build 43 adds
SD pin scope to the main navigation on Controller, LEDs, Speed test, Wi-Fi
setup, and SD Card. The scope page also links back to Controller and SD Card.
The existing SD Diagnostics entry and the complete scope feature remain.

The build and existing web smoke checks passed. The first OTA transfer was
interrupted while Build 42 remained operational; a retry installed Build 43.
Live checks verified the links on all five pages at desktop and mobile sizes,
navigation into the scope and its return link, and unchanged saved settings.
The previous capture was archived before the update. A fresh Wi-Fi-off set
retains all six 1,024-sample traces in RAM, with Wi-Fi and the original SD mount
restored and zero SD errors. Evidence and the archived previous capture are in
`build/diagnostics/sd-scope-install-20260921-095535/`.

## September 21: CLK pull-up clarification

The designer reiterated that the supplied print has no pull-up on SD CLK.
The earlier suggestion to investigate a presumed CLK pull-up/path fault was
based on an incorrect assumption and is withdrawn. The existing PCB review
already identified five pull-ups, on CMD and DAT0-3; CLK is excluded.
`restoreSdScopePins()` configures CLK as an input with its internal pull-up
disabled after detaching the SD host. The captured decay is consistent with
that undriven node being sampled and is not a test of the driven SD clock.
This corrects interpretation and project notes; no firmware or hardware change
is needed to address the absence of a CLK pull-up.
