# Native ESP-IDF port

The native build entry is `main/app.cpp::app_main()`. No Arduino source is
compiled or linked. The root Arduino sources are retained for comparison.

| Function | Native implementation |
|---|---|
| Application and display scheduling | FreeRTOS tasks; `esp_timer` wakeups |
| Hall index | GPIO3 falling-edge ISR, one pulse per revolution |
| Four strip outputs | GPIO21/38/18/7, common GPIO42 clock |
| Image memory | PSRAM-preferred `heap_caps_malloc`; internal LED staging |
| Settings | `nvs_*` APIs, existing `display` namespace; SD INI backup |
| SD storage | `esp_vfs_fat_sdmmc_mount`, POSIX file I/O |
| Wi-Fi | `esp_wifi`, `esp_netif`, native event handlers; AP and optional station |
| Hostname discovery | Espressif mDNS component |
| Web pages / APIs | `esp_http_server`, embedded HTML/JS, cJSON |
| Firmware updates | `esp_ota_begin/write/end/set_boot_partition` |
| Serial logging | ESP-IDF console on UART0, 115200 baud |

`sdkconfig.defaults` enables octal PSRAM and the existing 16 MB partition map.
ESP-IDF's generated flash arguments say `dio` for the bootloader image even
with QIO selected: the bootloader enables quad mode during initialization.
The QIO Kconfig selection remains enabled.

## Playback and migration

The native port imports Arduino's typed NVS keys, including the original
arm-1 `startch` key when no separate `start1` exists. An older PCB revision is
migrated to four independent arms; GPIO35/36/37 remain reserved for memory.
The optional encoder input is unused. GPIO42 is the LED clock. GPIO1 is
assigned to motor speed PWM; this build does not implement motor PWM control.

The FSEQ parser uses the published
[FPP format](https://github.com/FalconChristmas/fpp/blob/master/docs/FSEQ_Sequence_File_Format.txt).
Compression table entries identify the first frame in a block and its
compressed byte length. The old reader treated the first field as an
uncompressed size; the native reader corrects that and supports multi-frame
zlib blocks. It validates tables, file lengths, frame counts, and sparse
ranges before allocating frame storage. Zstd remains unsupported.

The four connector positions remain 90 degrees apart when fewer arms are
enabled. `BoardPins::ArmReverse` preserves/configures image pixel order.
The user confirmed strip inputs are at the hub; `BoardPins::ArmInputAtHub`
anchors the optional center-to-tip brightness ramp to that physical end,
independently of image reversal. A playback frame freezes when paused,
while position rendering continues. Loss of the Hall signal blanks playback
after the position timeout. Stationary diagnostics work without rotation.

## Web interface

The new pages replace the Arduino-generated HTML. `/`, `/files`, `/updates`,
`/ota`, and `/logs` serve the embedded interface with the corresponding
section selected. `/api/files` returns the directory listing as JSON.

`/wifi` serves a separate Wi-Fi setup page. `POST /wifi/scan` queues an
on-demand scan on the network task; `GET /wifi/scan` polls its status/results.
Scan results are limited to the strongest 32 access points and deduplicated by
SSID/security. Router connection attempts are serialized with scans. An
AP-only controller temporarily enables STA for scanning and then restores AP
mode, without initiating a router connection. Actual radio behavior still
requires testing on the PCB.

Router and access-point password fields support Show/Hide. A reveal of a saved
password uses `POST /wifi/password` with `target=router` or `target=ap`; the
response is not cached. Typing a new password and revealing it is entirely
local to the browser. Passwords are not included in HTML, scan results, logs,
or `/status`. A blank omitted password keeps the saved value only for the
same SSID; an explicit empty `pass` clears it for an open network.

Existing diagnostic URLs are retained: `/status`, `/diag/spi`, `/diag/timing`,
`/diag/blank`, `/diag/duty`, `/diag/map`, `/fseq/header`, `/fseq/cblocks`, and
`/fseq/ranges`. Their JSON reflects the native implementation. `/logs.txt`
returns the application's bounded log history. `/diag/reset` resets output
timing counters.

Settings POSTs accept URL parameters or URL-encoded request bodies. File
upload uses a **raw binary body** at `/upload?path=/name.fseq`. `/ota` receives
the raw application binary and restarts after validation. `/fw/upload` saves
the raw binary as `/firmware.bin` on SD for installation at the next boot.
The new page sends this format automatically; the old multipart upload forms
are not compatible. File transfers stop playback. Failed uploads discard
their temporary file, and failed direct OTA does not select the new partition.

Path validation keeps file operations inside `/sdcard`, rejects traversal,
and escapes filenames by creating DOM text nodes. Delete supports files and
empty directories, with no recursive deletion. Wi-Fi passwords are never
included in status JSON.

## Timing and verification

Normal playback uses `esp_lcd` I80 and GDMA at 20 MHz, driving all four strips
in parallel with one byte of lane levels per shared clock. Frame loading and
decompression run outside the display lock; completed frames are published by
swapping buffers and are adopted at spoke boundaries. A 100 ms frame-load
timeout blanks/stops playback without waiting for the reader, and late reads
cannot restart it. Remount waits until the cancelled read releases the card.
The stationary DMA benchmark runs
10 GPIO transfers and 100 DMA transfers, then blanks the strips and restores
GPIO routing. `/diag/dma` and the web DMA speed test show retained results.

DMA timing includes separate packing, submit-to-completion-interrupt, and
total call duration measurements. The color staging buffer or prebuilt black
buffer remains owned by the driver until completion; the display task waits
on a semaphore, allowing other tasks to run. The benchmark measures transfers,
not CPU utilization or overall playback capacity. SD reads, pattern generation,
and lock waits are outside the timing interval. See [DMA_TEST.md](DMA_TEST.md)
for clock limits, GPIO allocation, timeout behavior, and hardware verification.

Local verification includes a full ESP-IDF 5.3.3 build, executable host tests
for the shared format/protocol code, and Microsoft Edge browser tests using
mock device responses. These checks do not establish electrical pin mapping,
PSRAM presence, SD reliability, OTA operation, or speed on the physical board.

Bench sequence: flash with `build.ps1 -Action flash -Port COM29`, monitor the
startup log, check for 8 MB PSRAM, open the web UI, run connector/Hall tests,
check SD files and FSEQ playback, then inspect `/diag/spi` and the CI waveform.
