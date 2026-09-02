// USB Mode — expose the SD card to a PC as a USB Mass Storage device.
//
// Chosen from the disk menu ("USB Mode after reboot"), which sets an NVS flag
// and reboots. app_main() reads that flag FIRST — before Bluetooth, before
// arduino, before the emulator — and if it is set, comes here and never
// returns. The flag is cleared the moment it is read, so anything that reboots
// the board (replugging USB, RESET, a crash) comes back up as the emulator.
// One-shot by construction: there is no way to get stuck in USB Mode.
//
// Why the whole emulator is skipped rather than "paused":
//   - The PC writes RAW SECTORS. If np2kai still held the FAT mounted, its
//     FATFS cache would go stale behind the host's back and the card would be
//     corrupted. Here the card is opened with sdmmc_card_init() only; no
//     filesystem is ever mounted on this side.
//   - The S3 has ONE USB PHY. Handing it to the TinyUSB device stack takes the
//     USB-Serial-JTAG console away (same trade-off as ENABLE_USB_HOST — see
//     BOARD.md). There is nothing left to print to, so there is nothing to run.
//   - The BLE controller's 44KB of internal DMA RAM is not spent, which leaves
//     the USB stack plenty of room.
//
// Speed: reads and writes both land at ~0.7 MB/s, measured against a 16MB
// file with a SHA256 check. That is the Full Speed USB ceiling (the S3 OTG
// core has no High Speed), not the card — the SD side still has headroom.
// Getting there needed two things, both kept below:
//   - multi-sector sdmmc_*_sectors() calls. One sector per call (what
//     SD_MMC.writeRAW() does) costs a command plus the card's internal program
//     time per 512 bytes and measured 0.09 MB/s.
//   - a write-back cache handed to a separate task, so the card write overlaps
//     the next USB transfer instead of stalling it.

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_memory_utils.h"
#include "esp_private/usb_phy.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/lp_system_struct.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "tusb.h"

extern "C" {
#include <compiler.h>    // np2kai types
#include <i286c/cpumem.h>
#include <font/font.h>      // fontrom (mem + FONT_ADRS)
#include <font/fontdata.h>  // fontdata_8: the 8x8 ANK face built into np2kai
extern UINT8 mem[];         // np2kai main memory (cpumem.c) — a static array,
                            // so fontrom is usable without any emulator init
}

#include "board_pins.h"

extern "C" int ets_printf(const char *fmt, ...);

// USB Mode has no USB-Serial-JTAG console (the PHY belongs to the device stack
// now), so ets_printf goes nowhere. The startup log is kept here and pushed out
// of the CDC port instead.
//
// Deliberately a LINEAR buffer that stops when full, not a ring: it is REPLAYED
// from the beginning every time a host opens the port (tud_cdc_line_state_cb).
// A ring consumed on the first drain loses exactly the lines worth reading —
// Windows probes a new CDC as it enumerates, so by the time a terminal attaches
// the interesting part is gone. That cost a debugging round on the SPI board.
//
// Sized for a startup log, not a session history: this is .bss, i.e. internal
// RAM that the write-back cache would otherwise have.
//
// The buffer is malloc'd on entry to USB Mode rather than declared as .bss:
// this file is linked into the emulator too, and 2-3KB of internal RAM that
// only USB Mode ever touches is 2-3KB the FM mixer and the SD driver do not
// have. The emulator runs with about 20KB of internal DMA heap free, so a
// static buffer here is not free at all.
#define LOGBUF_SIZE 2048
static char *s_logbuf = nullptr;
static volatile uint32_t s_log_len = 0;   // bytes stored (stops at LOGBUF_SIZE)
static volatile uint32_t s_log_sent = 0;  // how much of it the host has had
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;

static void msc_logf(const char *fmt, ...) {
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(tmp) - 1) {
        n = (int)sizeof(tmp) - 1;
    }
    ets_printf("%s", tmp);  // still useful if a console does exist
    if (!s_logbuf) {
        return;
    }
    taskENTER_CRITICAL(&s_log_mux);
    for (int i = 0; i < n && s_log_len < LOGBUF_SIZE; i++) {
        s_logbuf[s_log_len++] = tmp[i];
    }
    taskEXIT_CRITICAL(&s_log_mux);
}

// Pull ESP_LOGx output into the same buffer. The SD driver explains its own
// failures far better than a returned esp_err_t can, and in USB Mode there is
// no console to read them on, so route them to the CDC alongside our lines.
static int msc_log_vprintf(const char *fmt, va_list ap) {
    char tmp[160];
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    if (n <= 0) {
        return 0;
    }
    if (n > (int)sizeof(tmp) - 1) {
        n = (int)sizeof(tmp) - 1;
    }
    ets_printf("%s", tmp);
    if (!s_logbuf) {
        return n;
    }
    taskENTER_CRITICAL(&s_log_mux);
    for (int i = 0; i < n && s_log_len < LOGBUF_SIZE; i++) {
        s_logbuf[s_log_len++] = tmp[i];
    }
    taskEXIT_CRITICAL(&s_log_mux);
    return n;
}
// usb_image.cpp writes through this. Its own ets_printf would only reach the
// console, and by the time anything goes wrong in USB Mode the console has been
// moved off the socket - the CDC log buffer is the only way out.
extern "C" void usb_msc_log(const char *fmt, ...) {
    char tmp[160];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) {
        msc_logf("%s", tmp);
    }
}

#define printf msc_logf

// Provided by lcd_rgb.cpp / menu_disk.cpp's renderer.
extern "C" bool lcd_init(void);
extern "C" void lcd_menu_clear(void);
extern "C" void lcd_menu_line(int row, const char *s, uint16_t fg, uint16_t bg);
extern "C" void lcd_menu_flush(void);
extern "C" void panel_tab5_blank_early(void);
// usb_kbd.cpp. Safe to run alongside the device stack here: the keyboard is on
// the USB-A socket, which is the high-speed controller with its own UTMI PHY,
// while USB Mode's device stack is on the full-speed controller and the USB-C
// socket. They share nothing.
extern "C" void usb_kbd_init(void);
extern "C" void bt_hid_init(void);   // bt_hid.cpp: BLE HID host
extern "C" bool usb_kbd_pop(uint8_t *nk, uint8_t *dn);

#define COL_WHITE 0xFFFF
#define COL_BLACK 0x0000
#define COL_YELLOW 0xFFE0

// ---- boot flag -----------------------------------------------------------
// Shares the "pc98" namespace with the rest of the settings (menu_disk.cpp).
#define NVS_NS "pc98"
#define NVS_KEY_USBMSC "usbmsc"

// Which of the two USB modes this boot is in. SD hands the host the card
// itself; IMAGE goes one level down and hands it the DOS volume inside the
// mounted PC-98 hard disk image (usb_image.cpp).
#define USB_MODE_SD    1
#define USB_MODE_IMAGE 2
static int s_mode = USB_MODE_SD;

extern "C" bool usb_image_open(void);
extern "C" bool usb_image_ready(void);
extern "C" uint32_t usb_image_block_count(void);
extern "C" uint16_t usb_image_block_size(void);
extern "C" int32_t usb_image_read(uint32_t lba, void *buf, uint32_t bytes);
extern "C" int32_t usb_image_write(uint32_t lba, const void *buf, uint32_t bytes);
extern "C" void usb_image_flush(void);
extern "C" const char *usb_image_status(void);
extern "C" const char *usb_image_name(void);

// Put the FSLS PHY mapping back the way the hardware boots with.
//
// USB Mode swaps it (see usb_msc_run): the OTG controller takes FSLS PHY 0,
// which is the one wired to the USB-C socket, and USB-Serial-JTAG is pushed
// onto PHY 1, which goes nowhere. That register lives in LP_SYS - the
// always-on domain - so it SURVIVES a CPU reset. Leaving USB Mode by pressing
// RESET therefore came back with the console still banished and the OTG
// controller sitting unconfigured on the socket: Windows sees a device that
// fails its descriptor request, there is no serial port, and the only way back
// in is the BOOT button. Only a power cycle clears it on its own.
//
// So every normal boot restores the default explicitly, before anything else.
extern "C" void usb_msc_restore_phy_map(void) {
    LP_SYS.usb_ctrl.sw_hw_usb_phy_sel = 0;  // hardware default: USJ on PHY 0
}

extern "C" void usb_msc_request(int mode) {
    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nh) == ESP_OK) {
        nvs_set_u8(nh, NVS_KEY_USBMSC, (uint8_t)mode);
        nvs_commit(nh);
        nvs_close(nh);
    }
}

// Read the flag AND clear it in the same breath. Clearing here (rather than on
// the way out of USB Mode) is what makes the mode survive nothing: a replug, a
// RESET or a panic all land back in the emulator.
// Bring-up only: 1 = boot into USB Mode without arming it from the menu.
// One-shot: the marker below survives esp_restart() but not a power cycle, so
// the 25-second no-host guard in usb_msc_run() can hand the board back.
#define USB_MSC_FORCE 0
#if USB_MSC_FORCE
static RTC_NOINIT_ATTR uint32_t s_force_done;
#define USB_MSC_FORCE_MAGIC 0xA5A5F0F0u
#endif

extern "C" int usb_msc_boot_flag_take(void) {
#if USB_MSC_FORCE
    if (s_force_done == USB_MSC_FORCE_MAGIC) {
        s_force_done = 0;
        return 0;
    }
    s_force_done = USB_MSC_FORCE_MAGIC;
    return USB_MODE_SD;
#endif
    nvs_handle_t nh;
    uint8_t want = 0;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nh) != ESP_OK) {
        return 0;
    }
    if (nvs_get_u8(nh, NVS_KEY_USBMSC, &want) != ESP_OK) {
        want = 0;
    }
    if (want) {
        nvs_erase_key(nh, NVS_KEY_USBMSC);
        nvs_commit(nh);
    }
    nvs_close(nh);
    return (int)want;
}

// ---- card ----------------------------------------------------------------
static const uint32_t SECTOR_SIZE = 512;
// Sectors per write-back buffer, settled at startup from whatever internal DMA
// memory the RGB panel left behind (see usb_msc_run). 64 = 32KB: the card
// writes that in ~8ms while USB (Full Speed) needs ~29ms to deliver the next
// one, so the card time hides completely behind it. Smaller still works, just
// with less margin.
static uint32_t s_cache_sectors = 0;
static const int CACHE_BUFS = 2;
static const uint32_t FLUSH_IDLE_MS = 100;

static sdmmc_card_t *s_card = nullptr;

static uint8_t *s_cache[CACHE_BUFS] = {nullptr, nullptr};
static int s_fill = 0;
static bool s_have_buf = false;
static uint32_t s_lba = 0;
static uint32_t s_count = 0;
static volatile uint32_t s_last_write_ms = 0;
static volatile bool s_failed = false;

typedef struct {
    int idx;
    uint32_t lba;
    uint32_t count;
} wrjob_t;

static QueueHandle_t s_wr_queue = nullptr;
static SemaphoreHandle_t s_free_bufs = nullptr;
static SemaphoreHandle_t s_mux = nullptr;

static uint8_t *s_bounce = nullptr;  // for transfers TinyUSB hands us unaligned

// The wiring comes from the BSP rather than a pin table of our own: all four
// data lines go straight to the SoC here, with no expander holding D3 the way
// the ST7701S board did, and the BSP already knows the slot and its width.
extern "C" void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config);
extern "C" void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config);

static bool card_init_once(void) {
    sdmmc_host_t host = {};
    sdmmc_slot_config_t slot = {};
    bsp_sdcard_get_sdmmc_host(SDMMC_HOST_SLOT_0, &host);
    bsp_sdcard_sdmmc_get_slot(SDMMC_HOST_SLOT_0, &slot);

    if (sdmmc_host_init() != ESP_OK) {
        return false;
    }
    if (sdmmc_host_init_slot(host.slot, &slot) != ESP_OK) {
        return false;
    }
    s_card = (sdmmc_card_t *)calloc(1, sizeof(sdmmc_card_t));
    if (!s_card) {
        return false;
    }
    esp_err_t err = sdmmc_card_init(&host, s_card);
    if (err != ESP_OK) {
        printf("usb_msc: sdmmc_card_init failed: %s\n", esp_err_to_name(err));
        free(s_card);
        s_card = nullptr;
        return false;
    }
    return true;
}

// Nothing unmounts the card before the reboot that lands in this mode, so the
// emulator can still have been reading from it when the host controller was
// reset. A card left mid-transaction ignores the initialisation sequence, and
// a single attempt then reports no card at all - which showed up as USB Mode
// coming up empty and rebooting itself, on a card that was perfectly good, on
// some attempts and not others. By the second go the card has timed out
// whatever it was doing.
static bool card_init(void) {
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt) {
            sdmmc_host_deinit();
            vTaskDelay(pdMS_TO_TICKS(150));
            printf("usb_msc: retrying the card (attempt %d)\n", attempt + 1);
        }
        if (card_init_once()) {
            return true;
        }
    }
    return false;
}

// The only place that actually writes the card. Separated from the USB
// callback so a card write and the next USB transfer run at the same time.
static void writer_task(void *arg) {
    (void)arg;
    wrjob_t job;
    for (;;) {
        xQueueReceive(s_wr_queue, &job, portMAX_DELAY);
        esp_err_t err = sdmmc_write_sectors(s_card, s_cache[job.idx], job.lba, job.count);
        if (err != ESP_OK) {
            printf("usb_msc: write %u failed: %s\n", (unsigned)job.lba, esp_err_to_name(err));
            s_failed = true;
        }
        xSemaphoreGive(s_free_bufs);
    }
}

// Hand the filled buffer to the writer. Caller holds s_mux.
static void submit_locked(void) {
    if (s_count == 0) {
        return;
    }
    wrjob_t job = {s_fill, s_lba, s_count};
    xQueueSend(s_wr_queue, &job, portMAX_DELAY);
    s_fill = (s_fill + 1) % CACHE_BUFS;  // FIFO queue + single writer => round robin
    s_count = 0;
    s_have_buf = false;
}

// Wait until everything submitted has reached the card. Caller holds s_mux.
static void barrier_locked(void) {
    submit_locked();
    for (int i = 0; i < CACHE_BUFS; i++) {
        xSemaphoreTake(s_free_bufs, portMAX_DELAY);
    }
    for (int i = 0; i < CACHE_BUFS; i++) {
        xSemaphoreGive(s_free_bufs);
    }
    s_have_buf = false;
}

static void flush_cache(void) {
    xSemaphoreTake(s_mux, portMAX_DELAY);
    barrier_locked();
    xSemaphoreGive(s_mux);
}

// ---- TinyUSB MSC callbacks ----------------------------------------------
extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id, "PC-98   ", 8);
    memcpy(product_id, (s_mode == USB_MODE_IMAGE) ? "Disk Image      "
                                                  : "SD Card Reader  ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    if (s_mode == USB_MODE_IMAGE) {
        return usb_image_ready();
    }
    return s_card != nullptr && !s_failed;
}

extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    if (s_mode == USB_MODE_IMAGE) {
        // The volume's own logical sector size, not the card's. Reporting
        // anything else would misdescribe the FAT sitting there.
        *block_count = usb_image_block_count();
        *block_size = usb_image_block_size();
        return;
    }
    *block_count = s_card ? s_card->csd.capacity : 0;
    *block_size = s_card ? (uint16_t)s_card->csd.sector_size : (uint16_t)SECTOR_SIZE;
}

extern "C" bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun;
    (void)power_condition;
    (void)start;
    (void)load_eject;
    // "Safely remove hardware" on the host lands here.
    if (s_mode == USB_MODE_IMAGE) {
        usb_image_flush();
        return true;
    }
    flush_cache();
    return true;
}

extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    (void)lun;
    if (s_mode == USB_MODE_IMAGE) {
        const uint32_t bs = usb_image_block_size();
        if (!bs || (offset % bs) != 0) {
            return -1;
        }
        xSemaphoreTake(s_mux, portMAX_DELAY);
        const int32_t r = usb_image_read(lba + offset / bs, buffer, bufsize);
        xSemaphoreGive(s_mux);
        return r;
    }
    if (!s_card || (offset % SECTOR_SIZE) != 0 || (bufsize % SECTOR_SIZE) != 0) {
        return -1;
    }
    const uint32_t start = lba + offset / SECTOR_SIZE;
    const uint32_t count = bufsize / SECTOR_SIZE;

    xSemaphoreTake(s_mux, portMAX_DELAY);
    // Settle every outstanding write first. Tracking which sectors are in
    // flight would be faster, but reads barely interleave with a bulk copy and
    // getting it wrong hands the host stale data.
    barrier_locked();

    esp_err_t err;
    // The SDMMC driver wants a DMA-capable, 64-byte-aligned buffer; TinyUSB's
    // endpoint buffer is neither by contract. Left unaligned the driver falls
    // back to allocating its own bounce buffer and can fail with ESP_ERR_NO_MEM.
    if (esp_ptr_dma_capable(buffer) && ((uintptr_t)buffer & 63) == 0) {
        err = sdmmc_read_sectors(s_card, buffer, start, count);
    } else if (bufsize <= CFG_TUD_MSC_EP_BUFSIZE) {
        err = sdmmc_read_sectors(s_card, s_bounce, start, count);
        if (err == ESP_OK) {
            memcpy(buffer, s_bounce, bufsize);
        }
    } else {
        err = ESP_ERR_INVALID_SIZE;
    }
    xSemaphoreGive(s_mux);

    if (err != ESP_OK) {
        printf("usb_msc: read %u failed: %s\n", (unsigned)start, esp_err_to_name(err));
        return -1;
    }
    return (int32_t)bufsize;
}

extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void)lun;
    if (s_mode == USB_MODE_IMAGE) {
        const uint32_t bs = usb_image_block_size();
        if (!bs || (offset % bs) != 0) {
            return -1;
        }
        xSemaphoreTake(s_mux, portMAX_DELAY);
        const int32_t r = usb_image_write(lba + offset / bs, buffer, bufsize);
        xSemaphoreGive(s_mux);
        return r;
    }
    if (!s_card || s_failed || (offset % SECTOR_SIZE) != 0 || (bufsize % SECTOR_SIZE) != 0) {
        return -1;
    }
    const uint32_t start = lba + offset / SECTOR_SIZE;
    uint32_t count = bufsize / SECTOR_SIZE;
    const uint8_t *src = buffer;

    xSemaphoreTake(s_mux, portMAX_DELAY);
    if (s_count && start != s_lba + s_count) {  // no longer contiguous
        submit_locked();
    }
    if (s_count == 0) {
        s_lba = start;
    }
    while (count > 0) {
        if (!s_have_buf) {
            xSemaphoreTake(s_free_bufs, portMAX_DELAY);  // waits only if the card fell behind
            s_have_buf = true;
        }
        const uint32_t room = s_cache_sectors - s_count;
        const uint32_t n = (count < room) ? count : room;
        memcpy(s_cache[s_fill] + s_count * SECTOR_SIZE, src, n * SECTOR_SIZE);
        s_count += n;
        src += n * SECTOR_SIZE;
        count -= n;
        if (s_count == s_cache_sectors) {
            const uint32_t next = s_lba + s_count;
            submit_locked();
            s_lba = next;
        }
    }
    s_last_write_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_mux);
    return (int32_t)bufsize;
}

extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void)lun;
    (void)buffer;
    (void)bufsize;
    switch (scsi_cmd[0]) {
        case 0x35:  // SYNCHRONIZE CACHE (10) - no SCSI_CMD_ enum for it
            flush_cache();
            return 0;
        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}

// ---- USB descriptors -----------------------------------------------------
#define USB_VID 0x303A  // Espressif
#define USB_PID 0x4003  // TinyUSB convention: bit0 = CDC, bit1 = MSC

static tusb_desc_device_t const s_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // Composite (CDC + MSC), so the device declares the IAD class triple and
    // each function's class lives in its interface association.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_MSC, ITF_NUM_TOTAL };
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define EPNUM_MSC_OUT 0x03
#define EPNUM_MSC_IN 0x83
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static uint8_t const s_desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

static char const *s_desc_strings[] = {
    (const char[]){0x09, 0x04},  // 0: en-US
    "np2 espresso",              // 1: manufacturer
    "PC-98 SD Card Reader",      // 2: product
    "123456",                    // 3: serial
    "USB Mode console",          // 4: CDC interface
    "SD Card Reader",            // 5: MSC interface
};

// ---- CDC: console + the reset that makes reflashing possible --------------
static volatile uint32_t s_cdc_baud = 0;

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    (void)itf;
    s_cdc_baud = coding->bit_rate;
}

// The 1200bps touch. Opening the port at 1200 baud and dropping DTR is how
// esptool (and the Arduino IDE) ask a native-USB board to reboot for flashing.
// Here it just restarts: the USB Mode flag has already been consumed, so the
// board comes back as the emulator, where the console is USB-Serial-JTAG again
// and esptool's own reset works. That is the whole point — while the device
// stack owns the PHY there is no port for esptool to drive, and reflashing
// would mean holding BOOT and pressing RESET by hand.
//
// Do NOT be tempted to set RTC_CNTL_FORCE_DOWNLOAD_BOOT here to land straight
// in the ROM loader. That bit is in the RTC domain and survives a reset, so
// every subsequent boot goes to the ROM loader too and the application never
// runs again to clear it — only pulling the USB cable (a real power-on reset)
// gets the board back. Tried on hardware; it strands the board.
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    if (dtr) {
        s_log_sent = 0;  // a terminal just attached: replay the startup log
        return;
    }
    if (s_cdc_baud == 1200) {
        flush_cache();  // do not strand a half-written cache in RAM
        panel_tab5_blank_early();
        esp_restart();
    }
}

// Hand the log to the host, from wherever it has got to.
static void drain_log_to_cdc(void) {
    if (!tud_cdc_connected() || !s_logbuf) {
        return;
    }
    while (s_log_sent < s_log_len) {
        if (tud_cdc_write_available() == 0) {
            break;
        }
        tud_cdc_write_char(s_logbuf[s_log_sent++]);
    }
    tud_cdc_write_flush();
}

extern "C" uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&s_desc_device;
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return s_desc_config;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc[32];
    uint8_t chr_count;

    if (index >= sizeof(s_desc_strings) / sizeof(s_desc_strings[0])) {
        return NULL;
    }
    if (index == 0) {
        memcpy(&desc[1], s_desc_strings[0], 2);
        chr_count = 1;
    } else {
        const char *str = s_desc_strings[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            desc[1 + i] = str[i];
        }
    }
    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc;
}

// ---- screen --------------------------------------------------------------
// lcd_menu_line() draws from np2kai's CGROM at fontrom+0x80000. In USB Mode
// nothing has loaded FONT.ROM (and the card belongs to the host anyway), so
// fill that area from fontdata_8 — the 8x8 ANK face compiled into np2kai —
// doubling each row to 8x16. This is exactly what font_load() does before it
// overlays anything read from a file (font.c).
static void load_builtin_font(void) {
    const UINT8 *p = fontdata_8;
    UINT8 *q = fontrom + 0x80000;
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < 8; j++) {
            q[0] = p[0];
            q[1] = p[0];
            p += 1;
            q += 2;
        }
    }
}

// The three full-screen modes - this one, the disk image reader and
// GreaseWeazle Mode - are laid out the same way as the menu's own submenus, so
// that picking a row and landing on its screen does not look like arriving
// somewhere else:
//
//   row 0   the title, in yellow, worded exactly as the menu row that led here
//   row 2+  what is going on, in white
//   row 6+  anything that needs a warning
//   row 9+  how to get back
//
// All of it indented two spaces, which is where the menu's own text starts.
#define MODE_INDENT "  "

static void draw_screen(void) {
    char line[80];
    lcd_menu_clear();

    if (s_mode == USB_MODE_IMAGE) {
        lcd_menu_line(0, MODE_INDENT "Disk Image Reader Mode", COL_YELLOW, COL_BLACK);
        snprintf(line, sizeof(line), MODE_INDENT "Image: %.40s", usb_image_name());
        lcd_menu_line(2, line, COL_WHITE, COL_BLACK);
        if (usb_image_ready()) {
            snprintf(line, sizeof(line), MODE_INDENT "%.50s", usb_image_status());
            lcd_menu_line(3, line, COL_WHITE, COL_BLACK);
            const uint64_t mb = ((uint64_t)usb_image_block_count() * usb_image_block_size()) >> 20;
            snprintf(line, sizeof(line), MODE_INDENT "%u MB volume on the PC.", (unsigned)mb);
            lcd_menu_line(4, line, COL_WHITE, COL_BLACK);
            if (usb_image_block_size() != 512) {
                lcd_menu_line(6, MODE_INDENT "NOTE: not 512-byte sectors - Windows will",
                              COL_WHITE, COL_BLACK);
                lcd_menu_line(7, MODE_INDENT "probably refuse to mount it.",
                              COL_WHITE, COL_BLACK);
            }
        } else {
            snprintf(line, sizeof(line), MODE_INDENT "FAILED: %.48s", usb_image_status());
            lcd_menu_line(3, line, COL_WHITE, COL_BLACK);
        }
    } else {
        lcd_menu_line(0, MODE_INDENT "SD Card Reader Mode", COL_YELLOW, COL_BLACK);
        if (s_card) {
            const uint64_t mb = ((uint64_t)s_card->csd.capacity * s_card->csd.sector_size)
                                / (1024ULL * 1024ULL);
            snprintf(line, sizeof(line), MODE_INDENT "SD card: %u MB", (unsigned)mb);
            lcd_menu_line(2, line, COL_WHITE, COL_BLACK);
            lcd_menu_line(3, MODE_INDENT "The card is now a drive on the PC.",
                          COL_WHITE, COL_BLACK);
        } else {
            lcd_menu_line(2, MODE_INDENT "No SD card.", COL_WHITE, COL_BLACK);
        }
    }

    lcd_menu_line(9,  MODE_INDENT "ESC on the keyboard returns to the emulator.",
                  COL_WHITE, COL_BLACK);
    lcd_menu_line(10, MODE_INDENT "Eject the drive on the PC first.",
                  COL_WHITE, COL_BLACK);
    lcd_menu_flush();
}

// ---- entry ---------------------------------------------------------------
static void tusb_task(void *arg) {
    (void)arg;
    for (;;) {
        // Blocking, not tud_task(). tud_task() polls with a zero timeout, so
        // this was a busy loop at priority 5 - fine when USB Mode had nothing
        // else to run, but it starves the USB HOST stack (priority 2) that now
        // shares the board so ESC can get back out of here. The keyboard simply
        // never finished enumerating.
        tud_task_ext(UINT32_MAX, false);
    }
}

// Any key press is read here, not in the main loop, so that leaving USB Mode
// never depends on what the rest of it is doing.
static void escape_task(void *arg) {
    (void)arg;
    for (;;) {
        uint8_t nk, dn;
        while (usb_kbd_pop(&nk, &dn)) {
            printf("usb_msc: key %02x %s\n", nk, dn ? "down" : "up");
            if (dn && nk == 0x00) {   // NKEY ESC
                printf("usb_msc: ESC, returning to the emulator\n");
                if (s_mode == USB_MODE_IMAGE) {
                    usb_image_flush();
                } else {
                    flush_cache();
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                panel_tab5_blank_early();
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Never returns.
extern "C" void usb_msc_run(int mode) {
    s_mode = (mode == USB_MODE_IMAGE) ? USB_MODE_IMAGE : USB_MODE_SD;
    // Nothing above this point has logged anything yet, so the buffer can be
    // claimed here — out of the emulator's way for the whole time it runs.
    s_logbuf = (char *)malloc(LOGBUF_SIZE);
    printf("\n=== USB Mode (%s) ===\n",
           (s_mode == USB_MODE_IMAGE) ? "disk image reader" : "SD card reader");

    // LCD before SD: the ST7701S 3-wire init drives GPIO 1/2, the same pins the
    // SDMMC bus uses. LCD_Init() finishes with them and frees them for the card.
    // It also brings up the I2C bus the panel and the expanders share.
    printf("usb_msc: LCD init: %s\n", lcd_init() ? "OK" : "FAIL");
    load_builtin_font();

    printf("usb_msc: internal DMA heap after LCD: free=%u largest=%u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

    s_mux = xSemaphoreCreateMutex();
    s_wr_queue = xQueueCreate(CACHE_BUFS, sizeof(wrjob_t));
    s_free_bufs = xSemaphoreCreateCounting(CACHE_BUFS, CACHE_BUFS);
    s_bounce = (uint8_t *)heap_caps_aligned_alloc(64, CFG_TUD_MSC_EP_BUFSIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_mux || !s_wr_queue || !s_free_bufs || !s_bounce) {
        printf("usb_msc: cannot allocate the basics\n");
        lcd_menu_clear();
        lcd_menu_line(8, "        USB MODE: OUT OF MEMORY", COL_WHITE, COL_BLACK);
        lcd_menu_flush();
        for (;;) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    // The write-back cache is a speed optimisation, not a requirement, so take
    // whatever internal DMA memory is going rather than refusing to run. What
    // is left after the RGB panel's bounce buffers varies with the panel
    // config, and a fixed demand turned a slower card reader into no card
    // reader at all. 64 sectors (32KB) is the point past which the card write
    // is already fully hidden behind the USB transfer; 8 (4KB, one USB
    // transfer) still pipelines, just with no margin.
    for (uint32_t want = 64; want >= 8; want /= 2) {
        for (int i = 0; i < CACHE_BUFS; i++) {
            s_cache[i] = (uint8_t *)heap_caps_aligned_alloc(64, want * SECTOR_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        }
        if (s_cache[0] && s_cache[1]) {
            s_cache_sectors = want;
            break;
        }
        for (int i = 0; i < CACHE_BUFS; i++) {
            free(s_cache[i]);
            s_cache[i] = nullptr;
        }
    }
    if (!s_cache_sectors) {
        printf("usb_msc: not even 2x4KB of internal DMA memory left\n");
        lcd_menu_clear();
        lcd_menu_line(8, "        USB MODE: OUT OF MEMORY", COL_WHITE, COL_BLACK);
        lcd_menu_flush();
        for (;;) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    printf("usb_msc: write cache %u KB x%d\n", (unsigned)(s_cache_sectors * SECTOR_SIZE / 1024), CACHE_BUFS);

    // Let the SD driver talk while it probes the card, then put it back so the
    // USB stack does not fill the buffer with noise.
    esp_log_set_vprintf(msc_log_vprintf);
    static const char *const sd_tags[] = {"sdmmc_cmd", "sdmmc_common", "sdmmc_init", "sdmmc_sd", "sdmmc_io"};
    for (unsigned i = 0; i < sizeof(sd_tags) / sizeof(sd_tags[0]); i++) {
        esp_log_level_set(sd_tags[i], ESP_LOG_DEBUG);
    }
    // Image mode goes through the filesystem, so the card is mounted normally
    // rather than opened raw. The two are mutually exclusive by design: raw
    // sector writes underneath a mounted FAT would corrupt it.
    bool card_ok = (s_mode == USB_MODE_IMAGE) ? usb_image_open() : card_init();
    for (unsigned i = 0; i < sizeof(sd_tags) / sizeof(sd_tags[0]); i++) {
        esp_log_level_set(sd_tags[i], ESP_LOG_INFO);
    }

    if (s_mode == USB_MODE_IMAGE) {
        printf("usb_msc: image %s: %s\n", card_ok ? "ready" : "FAILED", usb_image_status());
    } else if (!card_ok) {
        printf("usb_msc: no SD card\n");
    } else {
        const uint64_t mb = ((uint64_t)s_card->csd.capacity * s_card->csd.sector_size) / (1024ULL * 1024ULL);
        printf("usb_msc: SD %s %u MB, %d sectors @ %d kHz\n", s_card->cid.name, (unsigned)mb, s_card->csd.capacity, s_card->max_freq_khz);
        xTaskCreate(writer_task, "sdwriter", 3072, nullptr, 5, nullptr);
    }

    draw_screen();

    // A way out that does not need the PC, brought up before the device stack:
    // the virtual serial port this mode offers is the only other one, and if
    // the host will not talk to it there is otherwise nothing left but pulling
    // the power. The USB-A socket is the high-speed controller with its own
    // UTMI PHY, so it shares nothing with the device side.
    printf("usb_msc: internal free before kbd=%u largest_dma=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    extern void (*g_kbd_log)(const char *fmt, ...);   // usb_kbd.cpp
    g_kbd_log = usb_msc_log;
    usb_kbd_init();


    // Nothing to serve? Then do not take the PHY. Bringing the device stack up
    // over an empty drive costs the USB-Serial-JTAG console for no gain, and
    // leaves a board that can only be recovered by guessing at a virtual COM
    // port. Show why, and hand it back.
    if (!card_ok) {
        printf("usb_msc: nothing to serve, returning to the emulator\n");
        for (int i = 0; i < 8; i++) {
            uint8_t nk, dn;
            while (usb_kbd_pop(&nk, &dn)) {
                if (dn) {
                    i = 8;   // any key, not just ESC: there is nothing to wait for
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        panel_tab5_blank_early();
        esp_restart();
    }

    // Take the PHY for the device stack. This is what costs the USB-Serial-JTAG
    // console: the internal full-speed PHY is shared, and it is now OTG's.
    //
    // The speed is stated rather than left UNDEFINED, and on the ESP32-P4 that
    // is not a detail. usb_phy.c carries a compatibility path for that chip:
    // in DEVICE mode with the speed UNDEFINED or HIGH it silently overrides the
    // requested PHY with UTMI, which is the high-speed controller on rhport 1 -
    // the USB-A host receptacle on this board. The device then comes up on a
    // connector nothing can plug into, the USB-C side stays dead, and the
    // USB-Serial-JTAG console survives (which is the visible tell that this has
    // happened, since taking the right PHY is supposed to kill it).
    usb_phy_config_t phy_conf = {};
    phy_conf.controller = USB_PHY_CTRL_OTG;
    phy_conf.target = USB_PHY_TARGET_INT;
    phy_conf.otg_mode = USB_OTG_MODE_DEVICE;
    phy_conf.otg_speed = USB_PHY_SPEED_FULL;
    usb_phy_handle_t phy_hdl;
    if (usb_new_phy(&phy_conf, &phy_hdl) != ESP_OK) {
        printf("usb_msc: usb_new_phy failed\n");
    }
    // Move the OTG controller onto FSLS PHY 0.
    //
    // The ESP32-P4 has TWO internal full-speed PHYs, and the default mapping is
    // USB-Serial-JTAG on PHY 0 and USB_WRAP (the OTG 1.1 controller) on PHY 1.
    // Only PHY 0 goes to the USB-C socket on this board - that is where the
    // console appears - so the device stack was coming up on a PHY with nothing
    // attached: enumeration never started, and the console stayed alive, which
    // is the tell (taking the right PHY is supposed to take the console with
    // it). Swapping them puts the device on the socket and moves the console to
    // the pins that go nowhere.
    //
    // Written here rather than through usb_wrap_ll_phy_select(): that function
    // is missing a break in its switch, so case 0 falls through to case 1 and
    // both indices end up selecting the same mapping.
    LP_SYS.usb_ctrl.sw_hw_usb_phy_sel = 1;   // software decides the mapping
    LP_SYS.usb_ctrl.sw_usb_phy_sel = 1;      // USB_WRAP -> PHY 0, USJ -> PHY 1

    // tusb_init() has a zero-argument form only when the legacy
    // CFG_TUSB_RHPORT0_MODE is defined. tusb_config.h uses the current
    // CFG_TUD_ENABLED form, so go straight to the function it wraps.
    const tusb_rhport_init_t rh_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    printf("usb_msc: internal free before tusb=%u largest_dma=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!tusb_rhport_init(0, &rh_init)) {
        // Nothing will ever enumerate, so there is no point waiting out the
        // twenty-five second timeout below to find that out. Hand the console
        // back and say why on the panel, where it can still be read.
        printf("usb_msc: tusb_rhport_init failed - no device stack\n");
        lcd_menu_clear();
        lcd_menu_line(0, "  USB Mode", COL_YELLOW, COL_BLACK);
        lcd_menu_line(2, "  The USB device stack would not start.", COL_WHITE, COL_BLACK);
        lcd_menu_line(3, "  Restarting.", COL_WHITE, COL_BLACK);
        lcd_menu_flush();
        vTaskDelay(pdMS_TO_TICKS(3000));
        usb_msc_restore_phy_map();
        panel_tab5_blank_early();
        esp_restart();
    }

    xTaskCreate(tusb_task, "tusb", 4096, nullptr, 5, nullptr);

    // The way out runs on its own task rather than in the loop below. That
    // loop also drains the log to the CDC and settles the write cache, and if
    // anything there ever blocks - or the host keeps the device busy enough to
    // starve it - the keyboard would stop being read, which is the one thing
    // that must not happen in a mode that has taken the console away.
    xTaskCreate(escape_task, "usbesc", 3072, nullptr, 6, nullptr);

    // Bluetooth last, deliberately. The way out of this mode is a key press and
    // it has to work on whichever keyboard is in use - a BLE one is connected
    // by bt_hid.cpp and nothing else, so without this ESC does nothing for
    // anyone not carrying a USB keyboard. But it used to run BEFORE the device
    // stack, and that was wrong twice over: it took a large bite out of the
    // internal DMA memory that tusb_rhport_init() then needed, and it blocks
    // for about two seconds bringing the co-processor up, all of it before the
    // PHY was even taken. When the memory did not stretch the device never
    // enumerated, the host saw nothing, and twenty-five seconds later this mode
    // rebooted itself - intermittently, because how much Bluetooth takes varies.
    //
    // The device stack is the entire point of the mode and Bluetooth is a
    // convenience, so Bluetooth gets what is left rather than first pick. It
    // already reports and carries on when it cannot start.
    printf("usb_msc: internal free before BT=%u largest_dma=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    bt_hid_init();

    printf("usb_msc: ready\n");
    // If no host ever enumerates, come back. USB Mode has just moved the
    // console off the socket, so a mode that cannot talk to a PC is a board
    // that cannot be reflashed without holding BOOT - the one failure worth
    // spending a reboot to avoid. Only a total absence of enumeration counts;
    // once a host has been seen, unplugging is normal and stays put.
    const int64_t t_start = esp_timer_get_time();
    bool ever_mounted = false;
    for (;;) {
        if (tud_mounted()) {
            ever_mounted = true;
        } else if (!ever_mounted && esp_timer_get_time() - t_start > 25000000) {
            printf("usb_msc: no host after 25s, returning to the emulator\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            panel_tab5_blank_early();
        esp_restart();
        }
        // Settle the write-back cache once the host goes quiet, so an
        // unexpected unplug loses at most FLUSH_IDLE_MS of data.
        if (s_count && ((uint32_t)(esp_timer_get_time() / 1000) - s_last_write_ms) > FLUSH_IDLE_MS) {
            flush_cache();
        }
        drain_log_to_cdc();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
