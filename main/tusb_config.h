// TinyUSB device-stack configuration for the SD card reader (USB Mode).
//
// This project uses the RAW `espressif/tinyusb` component, not `esp_tinyusb`.
// That is deliberate: esp_tinyusb defines CONFIG_TINYUSB_MSC_ENABLED, and
// arduino-esp32's cores/esp32/USBMSC.cpp is guarded by exactly that symbol —
// enabling it would compile BOTH stacks' tud_msc_*_cb / descriptor callbacks
// and the link would fail on duplicate symbols. The raw component ships no
// Kconfig at all, so arduino's USB code stays compiled out (its
// CONFIG_TINYUSB_ENABLED is never defined) and only our callbacks exist.
//
// CFG_TUSB_MCU is supplied by the component as -DCFG_TUSB_MCU=OPT_MCU_ESP32S3;
// do not define it here.
//
// Reached by the tinyusb component because the project's root CMakeLists adds
// main/ to that component's include path — see CMakeLists.txt.

#pragma once

#include "tusb_option.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG 0
// TinyUSB can log from an ISR, so the printf must be the ROM one.
#define CFG_TUSB_DEBUG_PRINTF esp_rom_printf

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED  // the S3 OTG core is Full Speed only
#define CFG_TUD_DWC2_SLAVE_ENABLE 1

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN TU_ATTR_ALIGNED(4)
#define CFG_TUD_ENDPOINT0_SIZE 64

// Mass storage plus a serial port. The CDC is not decoration: taking the PHY
// for the device stack kills the USB-Serial-JTAG console, and without a port
// esptool cannot reset the board either — reflashing while in USB Mode would
// mean holding BOOT and pressing RESET by hand. The CDC gives back both the
// log and the 1200bps-touch reset (see tud_cdc_line_state_cb).
#define CFG_TUD_MSC 1
#define CFG_TUD_CDC 1
// Small on purpose: the CDC only carries the startup log and the 1200bps
// touch, and every byte here comes out of the internal RAM the write-back
// cache wants.
#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_DFU 0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_NCM 0
#define CFG_TUD_BTH 0

// One SCSI transfer per callback, and a static buffer of this size sits in
// internal RAM for as long as the firmware runs — including the whole time the
// emulator is running and USB Mode is not. 4096 (8 sectors) cost 2KB more than
// the emulator could spare and brought the FM crackle back on the SPI board.
//
// 2048 = 4 sectors. Writes are unaffected: they are batched into the 32KB
// write-back cache before they ever reach the card, so the size of the USB
// chunk does not change the transfer size the card sees. Reads issue twice as
// many multi-sector reads, which the Full Speed USB ceiling hides.
#define CFG_TUD_MSC_EP_BUFSIZE 2048

#ifdef __cplusplus
}
#endif
