# Board pinout — ESP32-S3 + ST7701S RGB panel (for np2_espresso fork)

Target hardware: ESP32-S3 N16R8 (16MB flash / 8MB Octal PSRAM) carrying a 2.8"
480x640 IPS panel on ST7701S in RGB parallel mode, a GT911 capacitive touch
controller, a TCA9554 I/O expander and a microSD slot. USB native D+=20 / D-=19.
Pin assignments below were read out of the board vendor's reference demo and are
mirrored in `main/board_pins.h`, which is the single source of truth in code.

## LCD — ST7701S, RGB parallel, 480(H) x 640(V) IPS
- Bring-up = 3-wire SPI init sequence, THEN esp_lcd_rgb_panel takes over.
  - SPI init: MOSI/SDA=**1**, SCLK=**2**, CS=**EXIO3**, RST=**EXIO1**  (SPI2_HOST)
- RGB sync: HSYNC=**38**, VSYNC=**39**, DE=**40**, PCLK=**41**  (pclk 30MHz, pclk_active_neg=false)
- RGB565 data (16-bit):
  - B0=5, B1=45, B2=48, B3=47, B4=21
  - G0=14, G1=13, G2=12, G3=11, G4=10, G5=9
  - R0=46, R1=3, R2=8, R3=18, R4=17
- Timing: hsync back=10 front=50 pulse=8 / vsync back=18 front=8 pulse=2
- Framebuffer in PSRAM (fb_in_psram=true).
- Backlight: GPIO**6** via LEDC PWM (13-bit, 4kHz), ON level = 1.
- DISP_EN = -1 (none).

## I2C bus (shared: TCA9554 + GT911 touch)
- SCL=**7**, SDA=**15**, I2C_NUM_0, 400kHz.
- TCA9554PWR addr=0x20 (regs: input0x00 output0x01 pol0x02 config0x03).

## TCA9554 EXIO bit map (1..8)
- EXIO1 = LCD RST
- EXIO2 = Touch(GT911) RST
- EXIO3 = LCD CS (for the SPI init)
- EXIO4 = SD CS / DAT3
- EXIO8 = Buzzer (on/off)
- (EXIO5/6/7 unused)

## microSD — SDMMC 1-bit
- CLK=**2**, CMD=**1**, D0=**42**, D3=**EXIO4**.  (D1/D2 unused)
- NOTE: CLK/CMD (2,1) are SHARED with the ST7701S SPI-init pins → init LCD first, then SD.

## Touch — GT911 (capacitive), I2C (7/15), RST=EXIO2, INT=(check GT911.h if needed)

## Audio — no onboard DAC (only buzzer @ EXIO8). DECISION: EXTERNAL I2S DAC.
- External I2S DAC (MAX98357A). **Current assignment (`BRD_I2S_*` in `main/board_pins.h`):**
  - **I2S BCLK = GPIO43**  (was UART0 TX)
  - **I2S LRCLK/WS = GPIO44**  (was UART0 RX)  → MAX98357A **LRC**
  - **I2S DIN/DOUT = GPIO16**  → MAX98357A **DIN**
  - Wire MAX98357A: VIN=5V, GND, BCLK=43, LRC=44, DIN=16 (GAIN/SD per module default).
- **Consequence: taking 43/44 removes the UART0 console.** The board's serial is
  then the native USB (USB-Serial-JTAG), which in turn requires USB HOST mode to
  stay off — see the console/USB note below. Keyboard and mouse come in over BLE.
- Earlier assignment (superseded): BCLK=16 / WS=4 / DOUT=0, chosen when 43/44 were
  still the console. GPIO 0 and 4 are free again.
- `app_main()` drives all three pins low at reset before `audio_init()` runs, so the
  DAC does not pop on power-on; that code reads the same `BRD_I2S_*` defines.
- Onboard buzzer (EXIO8) stays available but is NOT the audio path.

## Display speed — the rotation blit is the whole story

The panel is portrait 480x640 and the PC-98 image is landscape 640x400, so every
frame is transposed from the np2 framebuffer (PSRAM) into the panel framebuffer
(PSRAM). Both live in PSRAM, so this is purely a cache-traffic problem, and the
original loop got it backwards: it walked the source one pixel at a time *down a
column* (`src[py * 640]`, a 1280-byte step), so all 307200 pixels missed the
cache and each pulled a 64-byte line to use 2 bytes of it.

Benchmarked on hardware (`LCD_BENCH` in `lcd_rgb.cpp`):

| | time |
|---|---|
| plain 614KB PSRAM write | 18.8 ms |
| plain 614KB PSRAM linear read | 12.9 ms |
| rotation, per-pixel column walk (old) | **309 ms** |
| rotation, 32x32 tiled (now) | **39.7 ms** |

That 309 ms was the "sluggish graphics": ~3 fps, no matter what the emulator did.
Tiling processes a 32x32 block at a time, so the source stays cache-resident for
all 32 columns of the tile — 32 line fetches per tile instead of 1024. 32 pixels
is 64 bytes, exactly one S3 cache line, and both the panel pitch (960) and every
tile origin are 64-byte aligned, so the writes stay full-line too (a partial line
write to PSRAM becomes a read-modify-write, which is what starves the scanout DMA
and produces the thin-red-bar collapse).

In the running system the blit measures ~43.7 ms against a ~32 ms floor for
moving 1.1 MB through PSRAM, so what is left is bandwidth, not the loop —
the RGB scanout DMA itself continuously reads ~23 MB/s of that budget.
Further gains would have to come from moving less data, not from a faster loop.

**Double buffering is not an option here**: a second panel framebuffer is another
614 KB and PSRAM is down to ~400 KB free once the emulator is up.

## Console / native USB — one PHY, two jobs
The S3 has a single USB PHY: it is EITHER a USB HID **host** (keyboard/mouse) OR
the USB-Serial-JTAG **console**, never both. With UART0's pins spent on I2S:
- `ENABLE_USB_HOST 0` (main.cpp) + `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` — the
  current setup. Console works over the native USB; input is BLE only.
- Flipping `ENABLE_USB_HOST` to 1 restores USB keyboard/mouse and costs the
  console entirely (there is no UART left to fall back to).
- Flashing: if the running firmware ever holds USB in host mode the board does
  not enumerate, so recovery is BOOT held + RESET tapped (download mode).

## USB Mode — the SD card as a drive on the PC (usb_msc.cpp)
The disk menu's last row, `USB Mode after reboot`, sets an NVS flag and
restarts. `app_main()` reads that flag first thing — before Bluetooth, before
arduino, before the emulator — and if set, hands the card to the PC as a USB
mass storage device and never returns. The flag is **cleared as it is read**, so
a replug, a RESET or a crash all come back as the emulator: one-shot by
construction, with no way to get stuck.

Nothing else runs in that mode, for three reasons:
- The PC writes **raw sectors**. If np2kai still held the FAT mounted, its FATFS
  cache would go stale behind the host's back. Here the card is opened with
  `sdmmc_card_init()` only; no filesystem is mounted on this side.
- One PHY (see above): the device stack takes it, so the USB-Serial-JTAG
  console goes away. The CDC below is what gives it back.
- The BLE controller's 44KB of internal DMA RAM is never spent.

**Raw `espressif/tinyusb`, not `esp_tinyusb`.** esp_tinyusb defines
`CONFIG_TINYUSB_MSC_ENABLED`, and arduino-esp32's `cores/esp32/USBMSC.cpp` is
guarded by exactly that symbol — enabling it compiles BOTH stacks' `tud_msc_*`
and descriptor callbacks and the link fails on duplicate symbols. The raw
component ships no Kconfig, so arduino's USB code stays compiled out
(`CONFIG_TINYUSB_ENABLED` never gets defined). `main/tusb_config.h` reaches that
component because the root `CMakeLists.txt` injects `main/` into its include
path.

**Composite CDC + MSC** (PID 0x4003), not MSC alone. The CDC carries the startup
log — buffered linearly and replayed whenever a host raises DTR, so the lines
from before enumeration survive Windows' own probe of the new port —
and gives esptool a port to drive. A 1200bps touch with DTR dropped simply calls
`esp_restart()`: the flag is already consumed, so the board returns to the
emulator where the USB-Serial-JTAG console works and esptool resets it normally.
**Do not set `RTC_CNTL_FORCE_DOWNLOAD_BOOT` to land in the ROM loader instead** —
that bit is in the RTC domain and survives a reset, so every later boot goes to
the ROM loader too and the application never runs again to clear it. Only
unplugging the cable recovers the board. Tried on hardware.

Speed is ~0.68 MB/s each way (16MB, SHA256 checked) — the Full Speed USB
ceiling, not the card. Two things get there: multi-sector `sdmmc_*_sectors()`
(one sector per call, what `SD_MMC.writeRAW()` does, measures 0.09 MB/s), and a
write-back cache handed to a separate task so the card write overlaps the next
USB transfer. The cache takes **whatever internal DMA memory the RGB panel left**
(32KB down to 4KB per buffer, 2 buffers) rather than a fixed amount: after
`lcd_init()` there is ~78KB free / 69KB largest, and a fixed demand once turned
a slower card reader into no card reader at all. The chosen size is printed on
the CDC at startup along with the heap figures.

Text on the panel comes from `fontdata_8`, the 8x8 ANK face compiled into
np2kai, doubled to 8x16 into `fontrom + 0x80000` — the same thing `font_load()`
does before it overlays anything read from a file. FONT.ROM cannot be used here:
the card belongs to the host. `mem[0x200000]` is a static array, so `fontrom` is
usable with no emulator init at all.

Measuring it: do it with the drive **idle**. Benchmarking while the host is
copying files to the same card over the same bus reads about a third of the real
figure, which is easy to mistake for a regression.

## Battery ADC: GPIO4 (free again).  USB-OTG: D+=20, D-=19.

## Port implications vs original np2_espresso
- Display: replace TFT_eSPI/ST7789(SPI) entirely with esp_lcd_rgb (ST7701S). Panel is 480x640 portrait; to show PC-98 640x400 at native res, use it as 640x480 landscape (rotate the framebuffer 90deg when blitting).
- SD: replace Arduino SD.h/SPIClass with IDF esp_vfs_fat SDMMC (1-bit, CLK2/CMD1/D0=42, D3 via EXIO4).
- Add: TCA9554 driver (I2C) for LCD RST/CS, SD CS, buzzer, and backlight sequencing.
- WiFi: explicitly OFF (sdkconfig). Bluetooth: **BLE ON** — see below.
- Dropping TFT_eSPI + Arduino SD may allow dropping arduino-esp32 entirely (timing is already IDF-native).
- USB keyboard/mouse (usb_kbd.cpp): code unchanged (native OTG 20/19), but OFF by
  default here — the native USB is the console now. Input is BLE (bt_hid.cpp).

## Bluetooth keyboard / mouse (bt_hid.cpp)

- **The ESP32-S3 radio is Bluetooth LE only — there is no BR/EDR ("Bluetooth
  Classic") transceiver on the chip.** Only BLE / HOGP peripherals can ever work
  here. A Classic-only keyboard or mouse is impossible in firmware; anything that
  pairs with a phone or tablet as a HID device is BLE and is fine.
- Stack: Bluedroid, BLE-only, GATT **client** + SMP (no GATTS, no Classic), plus
  IDF's `esp_hidh`. `bt_hid.cpp` does the GAP half (scan / pair / connect) itself.
- Pairing is automatic and needs no UI: the host scans for peripherals that
  advertise the HID service (0x1812) or a HID appearance (0x03Cx), connects to
  the first one, and bonds. Bonding keys live in NVS, so a power cycle
  reconnects silently. Scan duty cycle: 30 s at boot, then 10 s every ~30 s
  while a keyboard *or* mouse is still missing, and nothing at all once both
  are connected (the radio must not compete with the RGB scanout).
- Pairing method is Just Works (`ESP_IO_CAP_NONE`) — there is no way to show a
  passkey while `lcd_menu_line()` is a stub. A keyboard that demands MITM
  passkey entry will log `auth fail` on the UART0 console and not pair.
- Reports funnel into the same code as USB (`hid_input.h`): keyboards use the
  8-byte boot layout, mice are decoded from the device's own HID report
  descriptor (BLE mice report in *report* protocol, with 8/12/16-bit deltas
  depending on the sensor, so the field offsets have to be parsed per device).
- Up to 3 links (`BT_MAX_DEV` / `CONFIG_BT_ACL_CONNECTIONS`); one keyboard +
  one mouse is the design target.

### RAM — the hard part. Read before changing sdkconfig or the boot order.

Internal SRAM (~128 KB of heap, total) is the binding constraint: the BLE
controller takes ~44 KB of it and, unlike everything else here, **cannot use
PSRAM at all**. Worse, when its allocation fails `btdm_controller_init()` does
not return an error — it crashes in its own cleanup path (deleting semaphores it
never created), which presents as a boot loop right after the SD mount.

What actually made it fit, in order of how much each mattered:

1. **Boot order.** `bt_hid_init()` runs at the very top of `app_main()`, before
   `initArduino()`, the RGB panel and SDMMC. Measured largest contiguous
   internal DMA block: 31.7 KB from inside `emu_task` → **47 KB** from the top of
   `app_main()`. It is contiguity, not total free bytes, that decides this.
2. `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` 32768 → **98304**. The old value was
   itself the ceiling: the biggest block on offer was exactly the 32 KB reserve.
3. `CONFIG_ESP_MAIN_TASK_STACK_SIZE` 32768 → **8192**. A leftover from before
   `emu_task` (which has its own static 56 KB stack) existed; it was costing
   32 KB of the contiguous internal heap.
4. `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` 16384 → **4096**, so ordinary mallocs
   stop eating the internal pool. Data placement was profiled on this core and
   found not to affect speed.
5. RGB bounce buffers 20 → **10** lines (`ST7701S.c`). The driver allocates two,
   so this returned ~19 KB of internal DMA memory. First thing to raise back if
   the panel tears under load.
6. The audio ring buffer moved to PSRAM (`xStreamBufferCreateWithCaps`) — it is
   only touched from tasks, never an ISR, and the I2S DMA descriptors need that
   internal space more.
7. `CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST` + `CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY`
   push the Bluedroid **host** allocations to PSRAM, and the
   FREERTOS/HEAP/RINGBUF/SPI-ISR "place into flash" options free ~28 KB of IRAM.

**Do not set `CONFIG_SPI_FLASH_ROM_IMPL`.** It saves ~10 KB of IRAM but swaps in
the ROM flash driver, which lacks IDF's cache-disable coordination between the
two cores. With np2kai's multi-megabyte `.bss` in PSRAM, core 1 touching it
during a flash operation faults as a cache access error — observed as a panic
inside `bios_memclear()`.

Measured on hardware after all of the above: BLE host up at 783 ms, emulator
running with ~23 KB internal free, I2S audio initialised.

### Two IDF gotchas this port had to work around

- **`esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler)` is the
  application's job.** `esp_hid` does not register it. Without that line,
  `esp_hidh_init()` calls `esp_ble_gattc_app_register()` and then blocks forever
  on a semaphore only the GATTC registration event would release.
- **The controller must be brought up on the core it is pinned to.** With
  `CONFIG_BT_CTRL_PINNED_TO_CORE_0`, `esp_bt_controller_init()` has to be called
  from core 0; `bt_hid_init()` therefore does all bring-up inside the core-0
  worker task and blocks until it finishes.

Turning `CONFIG_BT_ENABLED=y` on also forces
`CONFIG_ARDUINO_SELECTIVE_COMPILATION`: arduino-esp32 3.3.x builds its own BLE
wrapper, and that wrapper does not compile against ESP-IDF v5.5 (`BLEDevice.cpp`
calls `esp_ble_gap_get_local_irk()`, an IDF 6.x API). Selective mode also
silently drops libraries that have no `ARDUINO_SELECTIVE_*` switch (ESP_I2S,
ESP_NOW, USB, Console...), which breaks the ones that include them — hence
`ESP_SR=n` and the explicit keep-list in `sdkconfig.defaults`.

Note that `initArduino()` logs `btMemRelease(): BT memory release failed:
ESP_ERR_INVALID_STATE`. That error is *wanted*: Arduino tries to hand the BT
controller's memory back to the heap because no Arduino sketch asked for
Bluetooth, and it fails only because our controller is already running. It is
also the second reason Bluetooth must start before `initArduino()`.
