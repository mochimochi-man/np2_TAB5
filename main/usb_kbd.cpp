// USB HID keyboard host for the PC-98 emulator (milestone M4).
//
// ESP32-S3 USB-OTG in host mode, HID boot keyboard. USB HID usage codes are
// translated to PC-98 key scancodes (NKEY_*) and queued; the emulator task
// drains the queue and calls keystat_keydown/keyup. USB host structure follows
// the Anemoia-ESP32 usb_controller.cpp reference (same hid_host component).
//
// This file also owns the transport-independent half of the input path (the
// usage->NKEY table, the key queue, the mouse accumulators) — see hid_input.h.
// bt_hid.cpp feeds the very same functions from Bluetooth LE.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"

extern "C" int ets_printf(const char *fmt, ...);

// Where this file's messages go. USB Mode moves the console off the USB socket
// to serve mass storage, so anything printed there afterwards is lost - it
// points this at its CDC log buffer instead, which is the only channel left.
extern "C" void (*g_kbd_log)(const char *fmt, ...) = nullptr;
#define KLOG(...) do { \
        if (g_kbd_log) { g_kbd_log(__VA_ARGS__); } else { ets_printf(__VA_ARGS__); } \
    } while (0)
#include "usb/hid_host.h"
#include "usb/usb_host.h"
#include "hid_input.h"
#include "esp_timer.h"

extern "C" {
    void keystat_keydown(unsigned char ref);
    void keystat_keyup(unsigned char ref);
    int  ets_printf(const char *fmt, ...);
}

// HID usage id -> PC-98 NKEY scancode. 0xff = unmapped. Filled in usb_kbd_init.
static uint8_t s_hid2nkey[256];

// Event queue: packed (nkey<<1)|down. Drained by the emulator task.
static QueueHandle_t s_kq;
static QueueHandle_t s_hid_evtq;

// Previous report, per transport (see HID_SRC_* in hid_input.h).
static uint8_t s_prevkeys[HID_SRC_MAX][6];
static uint8_t s_prevmod[HID_SRC_MAX];

// Set when Pause/Break is pressed: the emulator loop opens the disk swap menu.
volatile int g_menu_req = 0;
// Set by the disk menu's CPU row: the emulator loop applies the new multiple.
volatile int g_speed_req = 0;

// Real mouse (USB boot protocol or BLE report protocol). Read by
// mousemng_getstat() in platform_stubs.cpp.
volatile int16_t g_umouse_dx = 0, g_umouse_dy = 0;   // accumulated deltas
volatile uint8_t g_umouse_btn = 0xA0;                // uPD8255 active-low (0x80=L, 0x20=R)
volatile int     g_umouse_present = 0;               // refcount: USB + BLE mice

// Accumulate movement + latch buttons. buttons: bit0=L bit1=R bit2=M.
extern "C" void hid_mouse_delta(int dx, int dy, uint8_t buttons) {
    g_umouse_dx = (int16_t)(g_umouse_dx + dx);
    g_umouse_dy = (int16_t)(g_umouse_dy + dy);
    uint8_t btn = g_umouse_btn;
    if (buttons & 0x01) btn &= ~0x80; else btn |= 0x80;   // left  (active low)
    if (buttons & 0x02) btn &= ~0x20; else btn |= 0x20;   // right (active low)
    g_umouse_btn = btn;
}

// Boot mouse report. byte0: bit0=L bit1=R; byte1: dx (int8); byte2: dy.
extern "C" void hid_mouse_boot_report(const uint8_t *d, int len) {
    if (len < 3) return;
    hid_mouse_delta((int8_t)d[1], (int8_t)d[2], d[0]);
}

extern "C" void hid_mouse_attach(void) { g_umouse_present++; }
extern "C" void hid_mouse_detach(void) { if (g_umouse_present > 0) g_umouse_present--; }

static void kq_push(uint8_t nkey, uint8_t down) {
    if (nkey == 0xff || !s_kq) return;
    uint16_t ev = (uint16_t)((nkey << 1) | (down & 1));
    xQueueSend(s_kq, &ev, 0);
}

// Pop one key event for the emulator loop. Returns 1 if an event was returned.
extern "C" int usb_kbd_pop(uint8_t *nkey, uint8_t *down) {
    if (!s_kq) return 0;
    uint16_t ev;
    if (xQueueReceive(s_kq, &ev, 0) != pdTRUE) return 0;
    *nkey = (uint8_t)(ev >> 1);
    *down = (uint8_t)(ev & 1);
    return 1;
}

// Temporary: dump every keyboard report plus the PC-98 code its first two key
// slots map to, so a key that "does not type" can be traced either to the report
// itself or to the usage->NKEY table. Set to 0 for normal builds.
#define HID_KBD_TRACE 0

// HID usages that open the disk menu. Pause/Break is the classic one, but many
// compact keyboards (and most Bluetooth ones) simply do not have that key, so
// F11 and F12 open it too: the PC-98 keyboard stops at F10, which leaves those
// two with no emulated meaning to lose. PrintScreen is deliberately NOT in this
// list - it carries the PC-98 COPY key.
static bool hid_is_menu_key(uint8_t k) {
    return k == 0x48       // Pause / Break
        || k == 0x44       // F11
        || k == 0x45;      // F12
}

// Parse an 8-byte HID boot keyboard report and queue press/release deltas.
extern "C" void hid_kbd_report(int src, const uint8_t *d, int len) {
    if (len < 8 || (unsigned)src >= HID_SRC_MAX) return;
#if HID_KBD_TRACE
    // The keyboard streams idle (all-zero) reports continuously, and a serial
    // capture rarely overlaps the moment a key is pressed - so keep the reports
    // that actually carry something and re-print the list every few seconds.
    {
        static uint8_t hist[16][8];
        static int nhist = 0;
        static int64_t last_dump = 0;
        bool nonzero = false;
        for (int i = 0; i < 8; i++) if (d[i]) nonzero = true;
        if (nonzero && nhist < 16) { memcpy(hist[nhist], d, 8); nhist++; }
        int64_t now = esp_timer_get_time();
        if (nhist && (now - last_dump) > 5000000) {
            last_dump = now;
            ets_printf("--- kbd history (%d) ---\n", nhist);
            for (int i = 0; i < nhist; i++)
                ets_printf("  mod=%02x keys %02x %02x %02x %02x %02x %02x -> nkey %02x %02x\n",
                           hist[i][0], hist[i][2], hist[i][3], hist[i][4],
                           hist[i][5], hist[i][6], hist[i][7],
                           s_hid2nkey[hist[i][2]], s_hid2nkey[hist[i][3]]);
        }
    }
#endif
    uint8_t *prevkeys = s_prevkeys[src];
    // modifiers: L/R shift, L/R ctrl, L/R alt(->GRPH)
    static const struct { uint8_t bit, nkey; } mods[] = {
        { 0x02, 0x70 }, { 0x20, 0x70 },   // shift
        { 0x01, 0x74 }, { 0x10, 0x74 },   // ctrl
        { 0x04, 0x73 }, { 0x40, 0x73 },   // alt -> GRPH
    };
    uint8_t mod = d[0];
    for (unsigned i = 0; i < sizeof(mods) / sizeof(mods[0]); i++) {
        uint8_t now = mod & mods[i].bit, was = s_prevmod[src] & mods[i].bit;
        if (now && !was) kq_push(mods[i].nkey, 1);
        else if (!now && was) kq_push(mods[i].nkey, 0);
    }
    s_prevmod[src] = mod;
    // released keys (in prev, not in current)
    for (int i = 0; i < 6; i++) {
        uint8_t k = prevkeys[i];
        if (k <= 1) continue;
        bool still = false;
        for (int j = 2; j < 8; j++) if (d[j] == k) still = true;
        if (!still) kq_push(s_hid2nkey[k], 0);
    }
    // pressed keys (in current, not in prev)
    for (int j = 2; j < 8; j++) {
        uint8_t k = d[j];
        if (k <= 1) continue;
        bool had = false;
        for (int i = 0; i < 6; i++) if (prevkeys[i] == k) had = true;
        if (!had) {
            if (hid_is_menu_key(k)) {
                g_menu_req = 1;                     // opens the disk swap menu
            } else {
                kq_push(s_hid2nkey[k], 1);
            }
        }
    }
    for (int i = 0; i < 6; i++) prevkeys[i] = d[i + 2];
}

// ---- hid_host plumbing (mirrors Anemoia usb_controller.cpp) ----
typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t  event;
    void                    *arg;
} hid_evt_t;

static void hid_iface_cb(hid_host_device_handle_t handle,
                         const hid_host_interface_event_t event, void *arg) {
    uint8_t data[64];
    size_t len = 0;
    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        if (hid_host_device_get_raw_input_report_data(handle, data, sizeof(data), &len) == ESP_OK) {
            if (arg == (void *)HID_PROTOCOL_MOUSE) hid_mouse_boot_report(data, (int)len);
            else hid_kbd_report(HID_SRC_USB, data, (int)len);
        }
        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        if (arg == (void *)HID_PROTOCOL_MOUSE) { hid_mouse_detach(); KLOG("usb_kbd: mouse disconnected\n"); }
        hid_host_device_close(handle);
        break;
    default: break;
    }
}

static void hid_dev_event(hid_host_device_handle_t handle,
                          const hid_host_driver_event_t event, void *arg) {
    hid_host_dev_params_t p;
    if (hid_host_device_get_params(handle, &p) != ESP_OK) return;
    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        if (p.proto != HID_PROTOCOL_KEYBOARD && p.proto != HID_PROTOCOL_MOUSE) { return; }
        const hid_host_device_config_t cfg = { .callback = hid_iface_cb, .callback_arg = (void *)p.proto };
        hid_host_device_open(handle, &cfg);
        // Boot protocol => fixed reports we know how to parse (kbd 8B / mouse 3B).
        hid_class_request_set_protocol(handle, HID_REPORT_PROTOCOL_BOOT);
        hid_host_device_start(handle);
        if (p.proto == HID_PROTOCOL_MOUSE) { hid_mouse_attach(); KLOG("usb_kbd: mouse connected\n"); }
        else KLOG("usb_kbd: keyboard connected\n");
    }
}

static void hid_dev_cb(hid_host_device_handle_t handle,
                       const hid_host_driver_event_t event, void *arg) {
    const hid_evt_t e = { handle, event, arg };
    xQueueSend(s_hid_evtq, &e, 0);
}

static void hid_task(void *arg) {
    (void)arg;
    hid_evt_t e;
    for (;;)
        if (xQueueReceive(s_hid_evtq, &e, portMAX_DELAY))
            hid_dev_event(e.handle, e.event, e.arg);
}

static void usb_lib_task(void *arg) {
    const usb_host_config_t hc = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    if (usb_host_install(&hc) != ESP_OK) { KLOG("usb_kbd: usb_host_install FAIL\n"); vTaskDelete(NULL); return; }
    xTaskNotifyGive((TaskHandle_t)arg);
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

// HID usage -> PC-98 NKEY table + key queue. Shared by the USB and BLE hosts;
// whichever starts first builds it, the second call is a no-op.
extern "C" void hid_map_init(void) {
    if (s_kq) return;
    for (int i = 0; i < 256; i++) s_hid2nkey[i] = 0xff;
    // letters a..z (HID 0x04..0x1d) -> PC-98 codes
    static const uint8_t letters[26] = {
        0x1d,0x2d,0x2b,0x1f,0x12,0x20,0x21,0x22,0x17,0x23,0x24,0x25,0x2f, // a..m
        0x2e,0x18,0x19,0x10,0x13,0x1e,0x14,0x16,0x2c,0x11,0x2a,0x15,0x29  // n..z
    };
    for (int i = 0; i < 26; i++) s_hid2nkey[0x04 + i] = letters[i];
    // digits 1..9,0 (HID 0x1e..0x27) -> 0x01..0x0a
    for (int i = 0; i < 9; i++) s_hid2nkey[0x1e + i] = (uint8_t)(0x01 + i);
    s_hid2nkey[0x27] = 0x0a;                                  // 0
    s_hid2nkey[0x28] = 0x1c;  // Enter -> RETURN
    s_hid2nkey[0x29] = 0x00;  // ESC
    s_hid2nkey[0x2a] = 0x0e;  // Backspace
    s_hid2nkey[0x2b] = 0x0f;  // Tab
    s_hid2nkey[0x2c] = 0x34;  // Space
    s_hid2nkey[0x2d] = 0x0b;  // -  -> minus
    s_hid2nkey[0x2e] = 0x0c;  // ^ (JIS ^ key) -> PC-98 ^
    // Symbol keys follow the JIS layout (the connected keyboard is JIS-106):
    s_hid2nkey[0x2f] = 0x1a;  // @  (JIS: HID 0x2f is the @ key)
    s_hid2nkey[0x30] = 0x1b;  // [  (JIS: HID 0x30 is the [ key)
    s_hid2nkey[0x31] = 0x0d;  // US backslash -> yen (JIS uses 0x89; harmless if sent)
    s_hid2nkey[0x32] = 0x28;  // ]  (JIS: HID 0x32 is the ] key)
    s_hid2nkey[0x33] = 0x26;  // ;
    s_hid2nkey[0x34] = 0x27;  // : (JIS colon)
    s_hid2nkey[0x36] = 0x30;  // ,
    s_hid2nkey[0x37] = 0x31;  // .
    s_hid2nkey[0x38] = 0x32;  // /
    s_hid2nkey[0x87] = 0x33;  // Intl1 "_ろ" (RO) key -> PC-98 RO (ろ / _)
    s_hid2nkey[0x89] = 0x0d;  // Intl3 "¥|" key       -> PC-98 yen (0x5C = "\")
    // arrows
    s_hid2nkey[0x4f] = 0x3c;  // Right
    s_hid2nkey[0x50] = 0x3b;  // Left
    s_hid2nkey[0x51] = 0x3d;  // Down
    s_hid2nkey[0x52] = 0x3a;  // Up
    s_hid2nkey[0x49] = 0x38;  // Insert
    s_hid2nkey[0x4c] = 0x39;  // Delete
    s_hid2nkey[0x4a] = 0x3e;  // Home -> HOMECLR
    // F1..F10 (HID 0x3a..0x43) -> 0x62..0x6b
    for (int i = 0; i < 10; i++) s_hid2nkey[0x3a + i] = (uint8_t)(0x62 + i);
    // PC-98 function / special keys. HID sources verified by logging the actual
    // codes the connected JIS keyboard sends (VF1..VF5 intentionally omitted):
    s_hid2nkey[0x47] = 0x60;  // ScrollLock   -> STOP
    s_hid2nkey[0x46] = 0x61;  // PrintScreen  -> COPY
    s_hid2nkey[0x4d] = 0x3f;  // End          -> HELP
    s_hid2nkey[0x4b] = 0x36;  // PageUp       -> ROLL UP
    s_hid2nkey[0x4e] = 0x37;  // PageDown     -> ROLL DOWN
    s_hid2nkey[0x39] = 0x71;  // CapsLock     -> CAPS
    s_hid2nkey[0x88] = 0x72;  // Intl2 かな    -> KANA
    s_hid2nkey[0x8a] = 0x35;  // Intl4 変換    -> XFER (henkan)
    s_hid2nkey[0x8b] = 0x51;  // Intl5 無変換  -> NFER (muhenkan)
    // numeric keypad (HID usage -> PC-98 NKEY_KP_*)
    s_hid2nkey[0x54] = 0x41;  // KP /
    s_hid2nkey[0x55] = 0x45;  // KP *
    s_hid2nkey[0x56] = 0x40;  // KP -
    s_hid2nkey[0x57] = 0x49;  // KP +
    s_hid2nkey[0x58] = 0x1c;  // KP Enter -> RETURN
    s_hid2nkey[0x59] = 0x4a;  // KP 1
    s_hid2nkey[0x5a] = 0x4b;  // KP 2
    s_hid2nkey[0x5b] = 0x4c;  // KP 3
    s_hid2nkey[0x5c] = 0x46;  // KP 4
    s_hid2nkey[0x5d] = 0x47;  // KP 5
    s_hid2nkey[0x5e] = 0x48;  // KP 6
    s_hid2nkey[0x5f] = 0x42;  // KP 7
    s_hid2nkey[0x60] = 0x43;  // KP 8
    s_hid2nkey[0x61] = 0x44;  // KP 9
    s_hid2nkey[0x62] = 0x4e;  // KP 0
    s_hid2nkey[0x63] = 0x50;  // KP .

    s_kq = xQueueCreate(64, sizeof(uint16_t));
}

extern "C" void tab5_usb_host_power(bool on);  // panel_tab5.cpp

extern "C" void usb_kbd_init(void) {
    hid_map_init();

    // Power the USB-A port. On the Tab5 its 5V comes from an I/O expander pin
    // and the port is dead without it - which is invisible from the host stack:
    // enumeration simply never starts, and nothing reports an error.
    tab5_usb_host_power(true);
    vTaskDelay(pdMS_TO_TICKS(100));
    s_hid_evtq = xQueueCreate(10, sizeof(hid_evt_t));

    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    ulTaskNotifyTake(false, pdMS_TO_TICKS(1000));

    const hid_host_driver_config_t dc = {
        .create_background_task = true, .task_priority = 5, .stack_size = 4096,
        .core_id = 0, .callback = hid_dev_cb, .callback_arg = NULL,
    };
    if (hid_host_install(&dc) != ESP_OK) { KLOG("usb_kbd: hid_host_install FAIL\n"); return; }
    xTaskCreatePinnedToCore(hid_task, "hid_task", 4096, NULL, 2, NULL, 0);
    KLOG("usb_kbd: USB HID keyboard host ready\n");
}
