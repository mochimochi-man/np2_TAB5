// Shared HID -> PC-98 input plumbing.
//
// Both transports feed the same funnel: usb_kbd.cpp (USB-OTG host, HID boot
// protocol) and bt_hid.cpp (Bluetooth LE / HOGP). usb_kbd.cpp owns the state —
// the HID-usage-to-NKEY table, the key event queue drained by the emulator
// task, and the g_umouse_* mouse accumulators read by mousemng_getstat().

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Build the HID usage -> PC-98 NKEY table and create the key queue. Idempotent:
// whichever transport comes up first pays for it.
void hid_map_init(void);

// Transports are tracked separately: press/release deltas are derived by
// diffing against the previous report, so a USB and a BLE keyboard connected at
// the same time must not share that "previous" state or they would cancel each
// other's keys out.
#define HID_SRC_USB 0
#define HID_SRC_BT  1
#define HID_SRC_MAX 2

// Standard 8-byte keyboard report (modifiers, reserved, 6 key usages). This is
// the USB boot layout, and it is also what essentially every BLE keyboard sends
// as its keyboard input report.
void hid_kbd_report(int src, const uint8_t *d, int len);

// 3-byte boot mouse report: buttons, int8 dx, int8 dy.
void hid_mouse_boot_report(const uint8_t *d, int len);

// Pre-decoded mouse movement, for report-protocol devices whose deltas are not
// 8-bit. buttons: bit0 = left, bit1 = right, bit2 = middle.
void hid_mouse_delta(int dx, int dy, uint8_t buttons);

// Mouse presence is refcounted across transports: mousemng_getstat() reports a
// neutral (all buttons up, no movement) state while the count is zero, so a
// USB mouse unplugged while a BLE one is still paired keeps working.
void hid_mouse_attach(void);
void hid_mouse_detach(void);

#ifdef __cplusplus
}
#endif
