// microSD for the M5Stack Tab5, mounted at /sd.
//
// Replaces the S3 forks' SD_Init() (their board_bsp) and the mainline's Arduino
// SD.begin(). Everything here is the BSP's one-stop call, deliberately:
//
// bsp_sdcard_mount() passes an all-zero config to bsp_sdcard_sdmmc_mount(),
// which is what makes the BSP fill in its own defaults — including bringing up
// the on-chip LDO that powers the card. Filling the host and slot structs by
// hand (via bsp_sdcard_get_sdmmc_host() / bsp_sdcard_sdmmc_get_slot()) skips
// that: the card then gets no supply, and the log reads
//     ldo: The voltage value 0 is out of the recommended range
//     sdmmc_init_ocr: send_op_cond (1) returned 0x107
// which looks like a missing card rather than a missing volt.
//
// The mount point has to be "/sd" because dosio_sd.cpp opens every image
// through that prefix; it comes from CONFIG_BSP_SD_MOUNT_POINT in
// sdkconfig.defaults rather than being passed in here.

#include <string.h>

#include "bsp/esp-bsp.h"
#include "sdmmc_cmd.h"

extern "C" int ets_printf(const char *fmt, ...);

// Give up the card cleanly. This matters before any reboot that is going to
// re-initialise it: nothing here stops the emulator using the card first, so a
// restart can land in the middle of a multi-block read, and a card left
// mid-transaction does not answer the initialisation sequence afterwards. That
// is what made USB Mode report "no SD card" on a machine whose card was fine,
// and only on some attempts - it depended on what the card happened to be
// doing at the moment of the reset.
extern "C" void sd_unmount(void) {
    bsp_sdcard_unmount();
}

extern "C" bool sd_mount(void) {
    // Two goes, for the same reason: if the card was left busy by a previous
    // life it will not answer the first time, and by the second it has timed
    // out whatever it was doing.
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        ets_printf("sd: mount failed: 0x%x, retrying\n", (unsigned)err);
        bsp_sdcard_unmount();
        vTaskDelay(pdMS_TO_TICKS(150));
        err = bsp_sdcard_mount();
    }
    if (err != ESP_OK) {
        ets_printf("sd: mount failed: 0x%x\n", (unsigned)err);
        return false;
    }
    sdmmc_card_t *card = bsp_sdcard_get_handle();
    if (card) {
        const uint64_t mb = ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024ULL * 1024ULL);
        ets_printf("sd: %s %u MB, %d-bit bus @ %d kHz\n", card->cid.name, (unsigned)mb, card->log_bus_width, card->max_freq_khz);
    }
    return true;
}

extern "C" sdmmc_card_t *sd_get_card(void) {
    return bsp_sdcard_get_handle();
}
