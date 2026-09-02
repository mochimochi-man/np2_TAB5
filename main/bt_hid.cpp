// Bluetooth LE HID host (HOGP) for the PC-98 emulator - wireless keyboard/mouse.
//
// IMPORTANT HARDWARE LIMIT: the ESP32-S3 radio does Bluetooth LE only. It has no
// BR/EDR ("Bluetooth Classic") transceiver at all, so a Classic-only keyboard or
// mouse can never work on this board no matter what the firmware does. Anything
// sold as "Bluetooth 4.0/5.x low energy" or that pairs with a phone/tablet as a
// HID device is BLE and is what this file talks to.
//
// Flow: scan for peripherals advertising the HID service (0x1812) or a HID
// appearance -> connect -> esp_hidh does GATT service discovery and subscribes
// to the input report notifications -> reports land in bt_hidh_cb() and are fed
// into the SAME funnel as the USB host (hid_input.h), so the emulator side is
// unchanged. Bonding keys live in NVS, so pairing survives a power cycle and
// re-connection afterwards is automatic and silent.
//
// Pairing is "Just Works" (we advertise no display and no keypad), which is what
// mice and the large majority of BLE keyboards accept. A keyboard that insists
// on MITM passkey entry cannot be paired here: there is no way to show the user
// a 6-digit code (lcd_menu_line() is still a stub on this panel) nor to type it.
// Such a failure is logged as "auth fail" on the UART0 console.

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// esp_bt.h omitted: that is the local controller API, and with
// CONFIG_BT_CONTROLLER_DISABLED there is no local controller to talk to.
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_hidh.h"
#include "esp_hidh_bluedroid.h"

// The ESP32-P4 has no radio. The controller lives on the ESP32-C6 next to it
// and is reached over SDIO with ESP-Hosted, which hands Bluedroid a virtual
// HCI. Everything below this line is stack-native and identical to the S3
// forks; only the bring-up differs.
// esp_hosted_bluedroid.h has no extern "C" guard of its own, and this is
// a C++ file.
extern "C" {
#include "esp_hosted.h"
#include "esp_hosted_bluedroid.h"
}
#include "bsp/esp-bsp.h"
#include "esp_hidh_gattc.h"
#include "esp_hid_common.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "hid_input.h"
#include "esp_timer.h"
// Temporary keyboard report dump (see handle_input). 0 for normal builds.
#define BT_MOUSE_TRACE 0
#define BT_KBD_TRACE 0
extern "C" int ets_printf(const char *fmt, ...);

static const char *TAG = "bt_hid";

// How many BLE HID peripherals may be connected at once (one keyboard + one
// mouse is the design target; the third slot absorbs combo devices).
#define BT_MAX_DEV 3

// Scanning costs radio time on the same core the LCD/emulator ISRs live on, so
// it is not left running forever: aggressive while nothing is paired, then short
// windows spaced apart while we are still missing a device, then fully idle.

// ---------------------------------------------------------------------------
// HID report descriptor parsing (mouse only)
//
// USB mice are put into boot protocol by usb_kbd.cpp, which gives a fixed
// 3-byte report. BLE mice have no such luxury: HOGP devices report in report
// protocol, and the layout differs per device - 8-bit deltas on cheap mice,
// 12-bit or 16-bit on anything with a high-resolution sensor, buttons and wheel
// in varying places. So we parse the device's own report descriptor once at
// connect time and remember where the fields are, in bits.
// ---------------------------------------------------------------------------

typedef struct {
    bool     valid;
    uint8_t  map_index;
    uint8_t  report_id;
    int16_t  btn_bit;    // bit offset of button 1, -1 if the report has none
    uint8_t  btn_cnt;
    int16_t  x_bit, y_bit;
    uint8_t  x_size, y_size;
} mouse_layout_t;

#define MAX_LAYOUT 4

typedef struct {
    esp_hidh_dev_t *dev;
    esp_bd_addr_t   bda;
    bool            in_use;
    bool            has_kbd;
    bool            has_mouse;
    mouse_layout_t  layout[MAX_LAYOUT];
    // Keyboard input report layout, read from the device's own descriptor.
    // -1 = not found, fall back to the boot layout (mod, reserved, 6 keys).
    int16_t         kbd_mod_bit;
    int16_t         kbd_key_bit;
    uint8_t         kbd_key_cnt;
    uint8_t         kbd_report_id;
} bt_dev_t;

static bt_dev_t s_dev[BT_MAX_DEV];

// Pull nbits starting at bit offset bitoff out of a little-endian bit stream and
// sign-extend. HID packs fields LSB-first within each byte.
static int32_t get_bits(const uint8_t *d, int len, int bitoff, int nbits) {
    if (nbits <= 0 || nbits > 32) return 0;
    if ((bitoff + nbits) > (len * 8)) return 0;
    uint32_t v = 0;
    for (int i = 0; i < nbits; i++) {
        int b = bitoff + i;
        if (d[b >> 3] & (1u << (b & 7))) v |= (1u << i);
    }
    if (nbits < 32 && (v & (1u << (nbits - 1)))) v |= ~((1u << nbits) - 1);  // sign extend
    return (int32_t)v;
}

// Running bit position per report ID (a descriptor interleaves several reports).
typedef struct { uint8_t id; uint16_t bitpos; } rpos_t;

static uint16_t *rpos_get(rpos_t *t, int *n, uint8_t id) {
    for (int i = 0; i < *n; i++) if (t[i].id == id) return &t[i].bitpos;
    if (*n >= 8) return &t[0].bitpos;                 // pathological descriptor
    t[*n].id = id; t[*n].bitpos = 0;
    return &t[(*n)++].bitpos;
}

static mouse_layout_t *layout_for(bt_dev_t *s, uint8_t map_index, uint8_t report_id, bool create) {
    for (int i = 0; i < MAX_LAYOUT; i++)
        if (s->layout[i].valid && s->layout[i].map_index == map_index &&
            s->layout[i].report_id == report_id) return &s->layout[i];
    if (!create) return NULL;
    for (int i = 0; i < MAX_LAYOUT; i++) {
        if (s->layout[i].valid) continue;
        mouse_layout_t *l = &s->layout[i];
        l->valid = true; l->map_index = map_index; l->report_id = report_id;
        l->btn_bit = -1; l->btn_cnt = 0;
        l->x_bit = -1; l->y_bit = -1; l->x_size = 0; l->y_size = 0;
        return l;
    }
    return NULL;
}

// Walk the HID report descriptor and record, for every input report that carries
// Generic Desktop X and Y, where the buttons and the two axes sit.
static void parse_report_map(bt_dev_t *s, uint8_t map_index, const uint8_t *rm, int len) {
    uint16_t usage_page = 0;
    uint32_t rep_size = 0, rep_count = 0;
    uint8_t  report_id = 0;
    uint16_t usages[16]; int n_usages = 0;
    uint32_t usage_min = 0; bool have_min = false;
    rpos_t   rpos[8]; int n_rpos = 0;

    int i = 0;
    while (i < len) {
        uint8_t item = rm[i++];
        if (item == 0xfe) {                            // long item: skip payload
            if (i + 1 >= len) break;
            int sz = rm[i];
            i += 2 + sz;
            continue;
        }
        int    sz   = item & 0x03;
        if (sz == 3) sz = 4;                           // size code 3 means 4 bytes
        uint8_t type = (item >> 2) & 0x03;             // 0=main 1=global 2=local
        uint8_t tag  = (item >> 4) & 0x0f;
        if (i + sz > len) break;
        uint32_t val = 0;
        for (int b = 0; b < sz; b++) val |= (uint32_t)rm[i + b] << (8 * b);
        i += sz;

        if (type == 1) {                               // ---- global ----
            switch (tag) {
            case 0x0: usage_page = (uint16_t)val; break;   // Usage Page
            case 0x7: rep_size   = val; break;             // Report Size
            case 0x8: report_id  = (uint8_t)val; break;    // Report ID
            case 0x9: rep_count  = val; break;             // Report Count
            default: break;
            }
        } else if (type == 2) {                        // ---- local ----
            switch (tag) {
            case 0x0:                                      // Usage
                if (n_usages < (int)(sizeof(usages) / sizeof(usages[0])))
                    usages[n_usages++] = (uint16_t)val;
                break;
            case 0x1: usage_min = val; have_min = true; break;  // Usage Minimum
            default: break;
            }
        } else if (type == 0) {                        // ---- main ----
            if (tag == 0x8) {                          // Input
                uint16_t *bitpos = rpos_get(rpos, &n_rpos, report_id);
                bool is_data = (val & 0x01) == 0;      // Data (not Constant padding)
                bool is_var  = (val & 0x02) != 0;      // Variable (not Array)
                // ---- keyboard fields ----
                // The boot report (modifier, reserved, six usages) is only what a
                // keyboard sends in BOOT protocol. In report protocol - which is
                // what HOGP devices actually use - the layout is whatever the
                // descriptor says, and cheap keyboards leave the reserved byte
                // out entirely: [modifiers][key1][key2]... Assuming the boot
                // layout there swallows the FIRST key of every report, so a key
                // only registers while another one is already held down.
                if (usage_page == 0x07 && is_data) {
                    if (is_var && rep_size == 1 && rep_count == 8) {
                        // The 8 modifier bits (LeftCtrl..RightGui).
                        if (s->kbd_mod_bit < 0) {
                            s->kbd_mod_bit = (int16_t)*bitpos;
                            s->kbd_report_id = report_id;
                        }
                    } else if (!is_var && rep_size == 8) {
                        // The array of currently-pressed key usages.
                        if (s->kbd_key_bit < 0) {
                            s->kbd_key_bit = (int16_t)*bitpos;
                            s->kbd_key_cnt = (uint8_t)(rep_count > 6 ? 6 : rep_count);
                            s->kbd_report_id = report_id;
                        }
                    }
                }
                if (is_data && is_var) {
                    if (usage_page == 0x09) {          // Button page
                        mouse_layout_t *l = layout_for(s, map_index, report_id, true);
                        if (l && l->btn_bit < 0 && rep_size == 1) {
                            l->btn_bit = (int16_t)*bitpos;
                            l->btn_cnt = (uint8_t)(rep_count > 8 ? 8 : rep_count);
                        }
                    } else if (usage_page == 0x01) {   // Generic Desktop
                        for (uint32_t f = 0; f < rep_count; f++) {
                            uint16_t u = (f < (uint32_t)n_usages) ? usages[f]
                                       : (have_min ? (uint16_t)(usage_min + f)
                                                   : (n_usages ? usages[n_usages - 1] : 0));
                            if (u != 0x30 && u != 0x31) continue;    // X / Y only
                            mouse_layout_t *l = layout_for(s, map_index, report_id, true);
                            if (!l) continue;
                            int16_t off = (int16_t)(*bitpos + f * rep_size);
                            if (u == 0x30) { l->x_bit = off; l->x_size = (uint8_t)rep_size; }
                            else           { l->y_bit = off; l->y_size = (uint8_t)rep_size; }
                        }
                    }
                }
                *bitpos = (uint16_t)(*bitpos + rep_size * rep_count);
            }
            // Every main item clears the local item state.
            n_usages = 0; have_min = false; usage_min = 0;
        }
    }

    // Drop half-parsed reports: without both axes it is not a mouse report.
    for (int k = 0; k < MAX_LAYOUT; k++) {
        mouse_layout_t *l = &s->layout[k];
        if (l->valid && (l->x_bit < 0 || l->y_bit < 0)) l->valid = false;
        else if (l->valid) {
            s->has_mouse = true;
            ESP_LOGI(TAG, "mouse report map=%u id=%u: btn@%d(%u) x@%d(%ub) y@%d(%ub)",
                     l->map_index, l->report_id, l->btn_bit, l->btn_cnt,
                     l->x_bit, l->x_size, l->y_bit, l->y_size);
        }
    }
}

// ---------------------------------------------------------------------------
// Device table
// ---------------------------------------------------------------------------

static bt_dev_t *dev_slot(esp_hidh_dev_t *dev) {
    for (int i = 0; i < BT_MAX_DEV; i++)
        if (s_dev[i].in_use && s_dev[i].dev == dev) return &s_dev[i];
    return NULL;
}

static int dev_count(void) {
    int n = 0;
    for (int i = 0; i < BT_MAX_DEV; i++) if (s_dev[i].in_use) n++;
    return n;
}

static bool dev_known_bda(const uint8_t *bda) {
    for (int i = 0; i < BT_MAX_DEV; i++)
        if (s_dev[i].in_use && memcmp(s_dev[i].bda, bda, ESP_BD_ADDR_LEN) == 0) return true;
    return false;
}

static volatile bool s_scanning = false;

// Console heartbeat. The worker spends most of its life inside a scan window or
// the pause after one, so the status has to be printed from inside those waits
// as well - otherwise it only appears every 30-50 seconds and looks absent.
static void bt_tick(void);
static void bt_sleep_ms(int ms) {
    while (ms > 0) {
        int chunk = ms > 500 ? 500 : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        ms -= chunk;
        bt_tick();
    }
}

// For the console heartbeat: is a keyboard / a mouse attached right now?
static bool s_have_kbd_now(void) {
    for (int i = 0; i < BT_MAX_DEV; i++) if (s_dev[i].in_use && s_dev[i].has_kbd) return true;
    return false;
}
static bool s_have_mouse_now(void) {
    for (int i = 0; i < BT_MAX_DEV; i++) if (s_dev[i].in_use && s_dev[i].has_mouse) return true;
    return false;
}

static void bt_tick(void) {
    // Only when something actually changes: a line every ten seconds forever is
    // noise on a console the user is reading too. Connects, disconnects and the
    // scan starting or stopping still show up the moment they happen.
    static int last_state = -1;
    const int state = (s_scanning ? 1 : 0) | (dev_count() << 1) |
                      (s_have_kbd_now() ? 0x100 : 0) | (s_have_mouse_now() ? 0x200 : 0);
    if (state == last_state) return;
    last_state = state;
    // dmafree is here because the SD driver allocates small internal DMA buffers
    // per transfer: if it approaches zero, disk reads start failing (and the
    // failure shows up as DOS sector errors, with nothing logged on this side).
    ets_printf("bt: scan=%d devices=%d kbd=%d mouse=%d bonds=%d dmafree=%u\n",
               (int)s_scanning, dev_count(),
               (int)s_have_kbd_now(), (int)s_have_mouse_now(),
               esp_ble_get_bond_device_num(),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
}

// True once we have both a keyboard and a mouse - nothing left to scan for.
static bool have_everything(void) {
    bool k = false, m = false;
    for (int i = 0; i < BT_MAX_DEV; i++) {
        if (!s_dev[i].in_use) continue;
        if (s_dev[i].has_kbd)   k = true;
        if (s_dev[i].has_mouse) m = true;
    }
    return k && m;
}

// ---------------------------------------------------------------------------
// esp_hidh events (run on the hidh event-loop task)
// ---------------------------------------------------------------------------

static void handle_open(esp_hidh_dev_t *dev) {
    bt_dev_t *s = NULL;
    for (int i = 0; i < BT_MAX_DEV; i++) if (!s_dev[i].in_use) { s = &s_dev[i]; break; }
    if (!s) { ESP_LOGW(TAG, "no free slot, closing"); esp_hidh_dev_close(dev); return; }

    memset(s, 0, sizeof(*s));
    s->in_use = true;
    s->dev = dev;
    s->kbd_mod_bit = -1;                 // "not found in the descriptor yet"
    s->kbd_key_bit = -1;
    const uint8_t *bda = esp_hidh_dev_bda_get(dev);
    if (bda) memcpy(s->bda, bda, ESP_BD_ADDR_LEN);

    // Which kinds of report does it send? The parsed report list carries the
    // usage esp_hidh will hand us on every input event.
    size_t nrep = 0;
    esp_hid_report_item_t *rep = NULL;
    if (esp_hidh_dev_reports_get(dev, &nrep, &rep) == ESP_OK && rep) {
        for (size_t i = 0; i < nrep; i++)
            if (rep[i].report_type == ESP_HID_REPORT_TYPE_INPUT &&
                rep[i].usage == ESP_HID_USAGE_KEYBOARD) s->has_kbd = true;
    }

    // Mouse field positions come from the raw descriptor (see parse_report_map).
    size_t nmaps = 0;
    esp_hid_raw_report_map_t *maps = NULL;
    if (esp_hidh_dev_report_maps_get(dev, &nmaps, &maps) == ESP_OK && maps)
        for (size_t m = 0; m < nmaps && m < 255; m++)
            if (maps[m].data && maps[m].len)
                parse_report_map(s, (uint8_t)m, maps[m].data, maps[m].len);

    // A new device just arrived. Nothing to arm here any more: the worker
    // keeps listening by itself until both a keyboard and a mouse are in, and
    // stops of its own accord once they are.
    if (s->has_mouse) hid_mouse_attach();
    if (s->has_kbd)
        ESP_LOGI(TAG, "kbd layout: id=%u mod@bit%d keys@bit%d x%u%s",
                 s->kbd_report_id, s->kbd_mod_bit, s->kbd_key_bit, s->kbd_key_cnt,
                 (s->kbd_key_bit < 0) ? " (descriptor had none: boot layout assumed)" : "");
    ESP_LOGI(TAG, "connected: %s [%02x:%02x:%02x:%02x:%02x:%02x]%s%s",
             esp_hidh_dev_name_get(dev) ? esp_hidh_dev_name_get(dev) : "(unnamed)",
             s->bda[0], s->bda[1], s->bda[2], s->bda[3], s->bda[4], s->bda[5],
             s->has_kbd ? " keyboard" : "", s->has_mouse ? " mouse" : "");
}

static void handle_close(esp_hidh_dev_t *dev) {
    bt_dev_t *s = dev_slot(dev);
    if (s) {
        // Release anything the emulator still thinks is held: a link that drops
        // mid-keypress would otherwise leave a key stuck down forever. An
        // all-zero report is exactly "no modifiers, no keys".
        if (s->has_kbd) { static const uint8_t none[8] = {0}; hid_kbd_report(HID_SRC_BT, none, 8); }
        if (s->has_mouse) hid_mouse_detach();
        ESP_LOGI(TAG, "disconnected [%02x:%02x:%02x:%02x:%02x:%02x]",
                 s->bda[0], s->bda[1], s->bda[2], s->bda[3], s->bda[4], s->bda[5]);
        s->in_use = false;
        s->dev = NULL;
    }
    esp_hidh_dev_free(dev);
}

static void handle_input(esp_hidh_dev_t *dev, esp_hid_usage_t usage, uint8_t map_index,
                         uint16_t report_id, const uint8_t *data, uint16_t len) {
    bt_dev_t *s = dev_slot(dev);
    if (!s || !data) return;

    if (usage == ESP_HID_USAGE_KEYBOARD) {
#if BT_KBD_TRACE
        // Keep the LAST 16 reports that carried anything (the keyboard streams
        // idle all-zero reports, and a key press is long over by the time a
        // serial capture starts) and re-print them every few seconds - with the
        // report id, the length and EVERY byte, including the one the boot
        // layout calls "reserved", which is where a non-standard keyboard can
        // hide keys.
        {
            static uint8_t hist[16][10];      // [0]=report_id [1]=len [2..9]=data
            static int nhist = 0, head = 0;
            static int64_t last_dump = 0;
            bool nonzero = false;
            for (int i = 0; i < len && i < 8; i++) if (data[i]) nonzero = true;
            if (nonzero) {
                hist[head][0] = (uint8_t)report_id;
                hist[head][1] = (uint8_t)len;
                for (int i = 0; i < 8; i++) hist[head][2 + i] = (i < len) ? data[i] : 0;
                head = (head + 1) % 16;
                if (nhist < 16) nhist++;
            }
            int64_t now = esp_timer_get_time();
            if (nhist && (now - last_dump) > 5000000) {
                last_dump = now;
                ets_printf("--- bt kbd reports (last %d) ---\n", nhist);
                for (int n = 0; n < nhist; n++) {
                    const uint8_t *e = hist[(head - nhist + n + 16) % 16];
                    ets_printf("  rid=%u len=%u : %02x %02x %02x %02x %02x %02x %02x %02x\n",
                               e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9]);
                }
            }
        }
#endif
        // Normalise to the boot layout the shared parser expects (modifier byte,
        // reserved byte, six key usages) using the offsets the device's own
        // descriptor gave us. A keyboard that omits the reserved byte reports
        // [modifiers][key1][key2]..., and reading that as a boot report loses
        // key1 - the key then only registers while another one is held.
        if (s->kbd_key_bit >= 0) {
            const int mod_off = (s->kbd_mod_bit >= 0) ? (s->kbd_mod_bit / 8) : -1;
            const int key_off = s->kbd_key_bit / 8;
            uint8_t norm[8] = { 0 };
            if (mod_off >= 0 && mod_off < len) norm[0] = data[mod_off];
            for (int i = 0; i < s->kbd_key_cnt && i < 6; i++) {
                int p = key_off + i;
                norm[2 + i] = (p < len) ? data[p] : 0;
            }
            hid_kbd_report(HID_SRC_BT, norm, 8);
            return;
        }
        // No usable descriptor: fall back to the boot layout.
        if (len >= 8) hid_kbd_report(HID_SRC_BT, data, 8);
        else ESP_LOGW(TAG, "short keyboard report (%u bytes), ignored", len);
        return;
    }

    if (usage != ESP_HID_USAGE_MOUSE && usage != ESP_HID_USAGE_GENERIC) return;

    const mouse_layout_t *l = layout_for(s, map_index, (uint8_t)report_id, false);
    if (!l) {
        // No descriptor match: fall back to the boot layout, which is what a
        // 3-byte mouse report almost certainly is.
        if (usage == ESP_HID_USAGE_MOUSE && len >= 3) hid_mouse_boot_report(data, len);
        return;
    }
    int32_t dx = get_bits(data, len, l->x_bit, l->x_size);
    int32_t dy = get_bits(data, len, l->y_bit, l->y_size);
    uint8_t btn = 0;
    if (l->btn_bit >= 0)
        for (int b = 0; b < l->btn_cnt && b < 8; b++)
            if (get_bits(data, len, l->btn_bit + b, 1) & 1) btn |= (uint8_t)(1u << b);
#if BT_MOUSE_TRACE
    // Rate-limited: a mouse streams reports far faster than a console can print,
    // and the raw bytes are what tells a report-ID offset error apart from a
    // sign/size error - so print both, a few times a second.
    {
        static int64_t last = 0;
        int64_t now = esp_timer_get_time();
        if (now - last > 300000) {
            last = now;
            ets_printf("mouse rid=%d len=%u raw %02x %02x %02x %02x %02x %02x %02x -> dx=%d dy=%d btn=%02x\n",
                       (int)report_id, (unsigned)len,
                       len > 0 ? data[0] : 0, len > 1 ? data[1] : 0, len > 2 ? data[2] : 0,
                       len > 3 ? data[3] : 0, len > 4 ? data[4] : 0, len > 5 ? data[5] : 0,
                       len > 6 ? data[6] : 0, (int)dx, (int)dy, btn);
        }
    }
#endif
    hid_mouse_delta((int)dx, (int)dy, btn);
}

static void bt_hidh_cb(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg; (void)base;
    esp_hidh_event_data_t *p = (esp_hidh_event_data_t *)data;
    switch ((esp_hidh_event_t)id) {
    case ESP_HIDH_OPEN_EVENT:  handle_open(p->open.dev); break;
    case ESP_HIDH_CLOSE_EVENT: handle_close(p->close.dev); break;
    case ESP_HIDH_INPUT_EVENT:
        handle_input(p->input.dev, p->input.usage, p->input.map_index,
                     p->input.report_id, p->input.data, p->input.length);
        break;
    case ESP_HIDH_BATTERY_EVENT:
        ESP_LOGI(TAG, "battery %u%%", p->battery.level);
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// GAP: scanning and pairing
// ---------------------------------------------------------------------------

typedef struct {
    esp_bd_addr_t bda;
    uint8_t       addr_type;
} found_t;

static QueueHandle_t s_found_q;          // scan hits, drained by the worker task
static volatile bool s_scan_ready = false;   // scan params accepted by the controller

static esp_ble_scan_params_t s_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    // 10 ms of listening in every 200, held open for as long as anything is
    // missing - in place of the old 40-in-50 burst that ran for twenty seconds
    // and then went deaf for a minute.
    //
    // Two things get better at once. A peripheral that wakes up advertises for
    // seconds, not minutes, so under the old pattern its entire burst could
    // fall in a gap and nothing would happen until it was nudged again; now
    // the window is never shut. And the radio uses a sixteenth of the airtime
    // it did, which is what the panel's scanout DMA and the SD card care about
    // - the old bursts were what made DOS report sector read errors.
    //
    // Sensitivity does not suffer the way the duty cycle suggests. A device
    // advertising every 50 ms sends twenty events a second and each has about
    // a one-in-twenty chance of falling in an open window, so it is seen
    // within roughly a second.
    .scan_interval      = 0x140,         // 200 ms
    .scan_window        = 0x10,          // 10 ms
    .scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE,
};

// Does this advertisement come from a HID peripheral? Either it lists the HID
// service in its UUID list, or its GAP appearance is a HID one (0x03Cx).
// The buffer passed in is the advertisement followed by the scan response, and
// both halves are searched: plenty of peripherals move the service UUID list
// into the scan response to make room in the 31-byte advertisement.
static bool adv_is_hid(uint8_t *adv, uint16_t adv_len) {
    uint8_t len = 0;
    uint8_t *p;

    p = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_16SRV_CMPL, &len);
    if (!p || !len)
        p = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_16SRV_PART, &len);
    for (int i = 0; p && (i + 1) < len; i += 2)
        if ((uint16_t)(p[i] | (p[i + 1] << 8)) == ESP_GATT_UUID_HID_SVC) return true;

    len = 0;
    p = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_APPEARANCE, &len);
    if (p && len >= 2) {
        uint16_t app = (uint16_t)(p[0] | (p[1] << 8));
        if ((app & 0xffc0) == 0x03c0) return true;    // 0x03C0..0x03C2: HID / keyboard / mouse
    }
    return false;
}

// Cached copy of the bond list. A bonded peripheral that wants its host back
// sends DIRECTED advertising, which carries NO advertising data at all - no
// service UUID, no appearance, nothing for adv_is_hid() to match on. Recognising
// it by address is the only way, and the list is cached because this is checked
// from the GAP callback on every scan hit.
static esp_bd_addr_t s_bond_cache[8];
static int           s_bond_cache_n = 0;

static void bond_cache_reload(void) {
    int n = esp_ble_get_bond_device_num();
    s_bond_cache_n = 0;
    if (n <= 0) return;
    if (n > 8) n = 8;
    esp_ble_bond_dev_t list[8];
    int cnt = n;
    if (esp_ble_get_bond_device_list(&cnt, list) != ESP_OK) return;
    for (int i = 0; i < cnt && i < 8; i++)
        memcpy(s_bond_cache[s_bond_cache_n++], list[i].bd_addr, ESP_BD_ADDR_LEN);
}

static bool bda_is_bonded(const uint8_t *bda) {
    for (int i = 0; i < s_bond_cache_n; i++)
        if (memcmp(s_bond_cache[i], bda, ESP_BD_ADDR_LEN) == 0) return true;
    return false;
}

static void bt_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Only now may scanning be started; the worker waits on this.
        s_scan_ready = true;
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        auto *r = &param->scan_rst;
        if (r->search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) { s_scanning = false; break; }
        if (r->search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;
        {
            const bool hid      = adv_is_hid(r->ble_adv, r->adv_data_len + r->scan_rsp_len);
            const bool directed = (r->ble_evt_type == ESP_BLE_EVT_CONN_DIR_ADV);
            const bool bonded   = bda_is_bonded(r->bda);
            if (!hid && !directed && !bonded) break;
            // Rate-limited: separates "the keyboard never advertises after a
            // reboot" from "we see it but the connection or the bond fails".
            static int64_t last_adv = 0;
            int64_t now = esp_timer_get_time();
            if ((now - last_adv) > 2000000) {
                last_adv = now;
                ets_printf("bt: adv %02x:%02x:%02x:%02x:%02x:%02x type=%d evt=%d rssi=%d%s%s%s\n",
                           r->bda[0], r->bda[1], r->bda[2], r->bda[3], r->bda[4], r->bda[5],
                           (int)r->ble_addr_type, (int)r->ble_evt_type, (int)r->rssi,
                           hid ? " hid" : "", directed ? " directed" : "",
                           bonded ? " bonded" : "");
            }
        }
        if (dev_known_bda(r->bda)) break;
        found_t f;
        memcpy(f.bda, r->bda, ESP_BD_ADDR_LEN);
        f.addr_type = r->ble_addr_type;
        if (s_found_q) xQueueSend(s_found_q, &f, 0);
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        s_scanning = false;
        break;

    // A HID peripheral asks the host to start encryption; always say yes. On a
    // re-connect to an already bonded device this is what silently restores the
    // link key, so no re-pairing is needed after a power cycle.
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        // Only reached if the peer forces a MITM method despite our "no IO"
        // capability. There is nowhere to show this on the panel yet.
        ESP_LOGW(TAG, "peer wants passkey pairing; passkey=%06u",
                 (unsigned)param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            // auth_mode tells a fresh pairing apart from a link resumed with the
            // stored keys, and the bond count says whether NVS actually kept it.
            ESP_LOGI(TAG, "auth ok: mode=0x%02x addr_type=%d bonds=%d",
                     param->ble_security.auth_cmpl.auth_mode,
                     (int)param->ble_security.auth_cmpl.addr_type,
                     esp_ble_get_bond_device_num());
            bond_cache_reload();      // a new bond must be recognisable at once
        } else
            ESP_LOGE(TAG, "auth fail, reason 0x%x - device may require passkey entry",
                     param->ble_security.auth_cmpl.fail_reason);
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Worker: owns scan windows and the (blocking) connect call
// ---------------------------------------------------------------------------

// Below this much contiguous internal DMA memory, btdm_controller_init() fails
// its own allocations - and then crashes in its cleanup path instead of
// returning an error (it deletes semaphores it never created), taking the whole
// firmware into a boot loop. So we refuse to even try when memory is that tight
// and let the emulator run without Bluetooth, which is a far better failure than
// a bricked-looking board.
//
// Measured on this board: 31744 crashes, 38912 and 47104 both work. Note how
// little it takes to move: adding ~2KB of statics elsewhere in the image dropped
// the largest block from 47104 to 38912, because what matters is contiguity, not
// free bytes. If this guard ever starts firing, that is the thing to chase.
#define BT_MIN_DMA_BLOCK 36864

// Bring up controller -> Bluedroid -> esp_hidh. Runs on core 0 (see bt_hid_init).
static bool bt_stack_up(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "internal heap before BT: free=%u largest_dma=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

    // The co-processor's supply is switched by the second I/O expander.
    bsp_i2c_init();
    bsp_feature_enable(BSP_FEATURE_WIFI, true);
    vTaskDelay(pdMS_TO_TICKS(200));

    if ((err = esp_hosted_init()) != ESP_OK) {
        ESP_LOGE(TAG, "esp_hosted_init failed: %s", esp_err_to_name(err)); return false;
    }
    // This is what resets the co-processor and runs the SDIO handshake, and it
    // is not optional: without it the transport task starts, no card is ever
    // found, and everything downstream simply waits.
    if (esp_hosted_connect_to_slave() != 0) {
        ESP_LOGE(TAG, "no link to the co-processor"); return false;
    }
    ESP_LOGI(TAG, "co-processor link up, attaching HCI");

    // Nothing attaches the hosted HCI driver implicitly - the Kconfig option
    // only compiles these functions in. Without this esp_bluedroid_enable()
    // waits forever for an HCI reset response that is never sent anywhere.
    hosted_hci_bluedroid_open();
    esp_bluedroid_hci_driver_operations_t hci_ops = {
        .send = hosted_hci_bluedroid_send,
        .check_send_available = hosted_hci_bluedroid_check_send_available,
        .register_host_callback = hosted_hci_bluedroid_register_host_callback,
    };
    if ((err = esp_bluedroid_attach_hci_driver(&hci_ops)) != ESP_OK) {
        ESP_LOGE(TAG, "attach_hci_driver failed: %s", esp_err_to_name(err)); return false;
    }

    ESP_LOGI(TAG, "starting bluedroid");
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(err)); return false;
    }
    ESP_LOGI(TAG, "bluedroid init ok, enabling");
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(err)); return false;
    }
    ESP_LOGI(TAG, "bluedroid enabled");

    esp_ble_gap_register_callback(bt_gap_cb);

    // Security: LE Secure Connections + bonding, no MITM (we have no display and
    // no keypad to run a passkey exchange with), 16-byte keys, exchange the
    // encryption and identity keys so a bonded device can reconnect with a
    // resolvable private address.
    esp_ble_auth_req_t  auth  = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t    iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE,   &auth,     sizeof(auth));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,        &iocap,    sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,      &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,      &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,       &rsp_key,  sizeof(rsp_key));

    // Address resolution. A HID peripheral advertises with a resolvable private
    // address that changes every few minutes, so without the privacy feature the
    // same keyboard looks like a brand new device on every reconnect - it gets
    // paired again from scratch instead of resuming the stored bond, which is
    // exactly the "pairing is not remembered" symptom.
    esp_ble_gap_config_local_privacy(true);

    // Report what is actually stored, so "it forgot the pairing" can be told
    // apart from "the keyboard itself re-pairs every time".
    {
        int nb = esp_ble_get_bond_device_num();
        ets_printf("bt_hid: bonded devices in NVS: %d\n", nb);
        if (nb > 0 && nb <= 8) {
            esp_ble_bond_dev_t list[8];
            int n = nb;
            if (esp_ble_get_bond_device_list(&n, list) == ESP_OK)
                for (int i = 0; i < n; i++)
                    ets_printf("  bond %d: %02x:%02x:%02x:%02x:%02x:%02x\n", i,
                               list[i].bd_addr[0], list[i].bd_addr[1], list[i].bd_addr[2],
                               list[i].bd_addr[3], list[i].bd_addr[4], list[i].bd_addr[5]);
        }
        // A HID peripheral that advertises with a rotating private address adds a
        // NEW bond every time it is paired again, so the (8 entry) list fills up
        // with dead copies of the same keyboard and then refuses new pairings.
        // Full list = nothing in it is usable anyway: wipe it and pair once more.
        // BT_CLEAR_BONDS_ON_BOOT: one-shot wipe for when a device connects and
        // exchanges its report map but then never sends an input report. A
        // peripheral with a rotating private address collects several bonds for
        // the same physical device, and a stale one being matched instead of the
        // live one is one way that ends up with a connection that carries no
        // notifications. Set to 1, boot once, pair once, then set back to 0.
#define BT_CLEAR_BONDS_ON_BOOT 0
        if (nb >= 8 || BT_CLEAR_BONDS_ON_BOOT) {
            esp_ble_bond_dev_t list[8];
            int n = 8;
            if (esp_ble_get_bond_device_list(&n, list) == ESP_OK)
                for (int i = 0; i < n; i++) esp_ble_remove_bond_device(list[i].bd_addr);
            ets_printf("bt_hid: bond list cleared (%d entries) - pair the devices once more\n", nb);
        }
    }

    // esp_hidh does NOT register its own GATTC callback - the application has to
    // hand it through. Without this, esp_hidh_init() calls
    // esp_ble_gattc_app_register() and then blocks forever on a semaphore that
    // only the GATTC registration event would release.
    esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);

    ESP_LOGI(TAG, "security params set, starting hidh");
    esp_hidh_config_t hcfg = {
        .callback         = bt_hidh_cb,
        .event_stack_size = 4096,
        .callback_arg     = NULL,
    };
    if ((err = esp_hidh_init(&hcfg)) != ESP_OK) {
        ESP_LOGE(TAG, "hidh init failed: %s", esp_err_to_name(err)); return false;
    }
    ESP_LOGI(TAG, "hidh up, setting scan params");

    bond_cache_reload();   // directed-advertising reconnects are matched by address
    esp_ble_gap_set_scan_params(&s_scan_params);
    ESP_LOGI(TAG, "BLE HID host ready (LE only - Bluetooth Classic devices are not supported)");
    return true;
}

static SemaphoreHandle_t s_up_sem;        // signals bring-up done (ok or failed)

static void bt_worker(void *arg) {
    (void)arg;

    bool ok = bt_stack_up();
    if (s_up_sem) xSemaphoreGive(s_up_sem);
    if (!ok) { vTaskDelete(NULL); return; }

    // esp_ble_gap_start_scanning() fails until the controller has taken the scan
    // parameters, which happens asynchronously in the GAP callback.
    for (int w = 0; w < 100 && !s_scan_ready; w++) vTaskDelay(pdMS_TO_TICKS(50));
    if (!s_scan_ready) { ESP_LOGE(TAG, "scan params never accepted"); vTaskDelete(NULL); return; }

    for (;;) {
        bt_tick();

        // Everything wanted is attached: stop listening altogether. This is the
        // state the emulator spends its life in, and a silent radio is what the
        // panel and the SD card want.
        if (have_everything() || dev_count() >= BT_MAX_DEV) {
            if (s_scanning) {
                esp_ble_gap_stop_scanning();
                for (int w = 0; w < 25 && s_scanning; w++) vTaskDelay(pdMS_TO_TICKS(20));
                s_scanning = false;
                xQueueReset(s_found_q);
            }
            bt_sleep_ms(2000);
            continue;
        }

        // Something is missing, so listen - continuously, at the low duty the
        // scan parameters set. There is no retry cadence to fall out of step
        // with: whenever a peripheral advertises, the window is already open.
        if (!s_scanning) {
            s_scanning = true;
            if (esp_ble_gap_start_scanning(0 /* no time limit */) != ESP_OK) {
                s_scanning = false;
                bt_sleep_ms(2000);
                continue;
            }
        }

        found_t f;
        if (xQueueReceive(s_found_q, &f, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        if (dev_known_bda(f.bda)) {
            continue;
        }

        // Scanning and connecting at the same time upsets the controller, so
        // close the window first and let it settle.
        esp_ble_gap_stop_scanning();
        for (int w = 0; w < 25 && s_scanning; w++) vTaskDelay(pdMS_TO_TICKS(20));
        s_scanning = false;
        xQueueReset(s_found_q);

        ESP_LOGI(TAG, "connecting to %02x:%02x:%02x:%02x:%02x:%02x",
                 f.bda[0], f.bda[1], f.bda[2], f.bda[3], f.bda[4], f.bda[5]);
        // Blocking: returns once the link is up and GATT discovery is done.
        // The OPEN event has already been posted by then.
        esp_hidh_dev_open(f.bda, ESP_HID_TRANSPORT_BLE, f.addr_type);
    }
}

// ---------------------------------------------------------------------------

// Start the BLE HID host. Two ordering requirements:
//   - AFTER nvs_flash_init(): the bonding keys are kept in NVS.
//   - BEFORE lcd_init()/SD_Init(): the BLE controller needs a big contiguous
//     block of INTERNAL DMA memory, and the RGB panel's bounce buffers plus the
//     SDMMC driver leave only ~31 KB of it - not enough, and the controller
//     answers that by crashing rather than by returning an error.
//
// The bring-up itself runs on CORE 0 (the emulator owns core 1 and must not be
// preempted by radio work), and this call blocks until it is done so that the
// controller gets first claim on memory before the display comes up.
extern "C" void bt_hid_init(void) {
    hid_map_init();
    s_found_q = xQueueCreate(8, sizeof(found_t));
    if (!s_found_q) { ESP_LOGE(TAG, "queue alloc failed"); return; }
    s_up_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(bt_worker, "bt_hid", 6144, NULL, 3, NULL, 0);
    if (s_up_sem) {
        xSemaphoreTake(s_up_sem, pdMS_TO_TICKS(10000));
        vSemaphoreDelete(s_up_sem);
        s_up_sem = NULL;
    }
}
