// Disk image swap menu (runtime FDD1/FDD2/HDD exchange).
//
// Modal text UI on the ST7789. Entered via the Pause/Break key (g_menu_req is
// set in usb_kbd.cpp); the emulator loop calls menu_disk_run() and pauses
// pccore_exec while it is active. Runs on the emulator task (core 1), so SD
// file operations happen in the same safe context as the boot-time mounts
// (fopen inside pccore_exec crashes — that is why the delayed setfdd path is
// not used).

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"

extern "C" int ets_printf(const char *fmt, ...);
#include <sys/stat.h>
#include <ctype.h>
#include <nvs.h>
#include <esp_system.h>   // esp_restart() for the RESET menu item
#include <dirent.h>       // POSIX opendir/readdir over the SD VFS ("/sd")

extern "C" {
#include <compiler.h>
#include <pccore.h>
#include <diskdrv.h>
#include <dosio.h>
#include <fdd/sxsi.h>
#include <fdd/newdisk.h>
#include "board_pins.h"
int  ets_printf(const char *fmt, ...);
int  usb_kbd_pop(uint8_t *nkey, uint8_t *down);   // usb_kbd.cpp
}

extern volatile int g_speed_req;    // main.cpp applies the new CPU multiple

// LCD text helpers (lcd_st7789.cpp)
extern "C" void lcd_menu_clear(void);
extern "C" void lcd_menu_line(int row, const char *s, uint16_t fg, uint16_t bg);
extern "C" void lcd_menu_flush(void);
extern "C" void lcd_menu_blank_rows(int first, int last);
extern "C" void panel_tab5_blank_early(void);  // panel_tab5.cpp: backlight + panel rail off
extern "C" void tab5_backlight_set(int percent);   // panel_tab5.cpp
extern "C" void audio_set_volume(int percent);     // audio_codec.cpp
extern "C" int  audio_get_volume(void);

// Backlight and volume are both a percentage in steps of 10. Kept out of the
// emulator core: neither is a PC-98 property, they are this tablet's.
#define LEVEL_STEP 10
#define LEVEL_MIN  10
extern "C" int g_backlight_pct;   // main.cpp owns it so boot can apply it

// RET wraps around; the arrow keys nudge, which is what anyone actually wants
// when the far end of the range is one step in the wrong direction.
// The three rows that hold a value - CPU clock, backlight, volume - all behave
// the same way:
//
//   LEFT / RIGHT   step down or up, and STOP at the ends
//   RET            step up, and wrap round to the bottom at the top
//
// The split matters rather than being tidiness. Left and right are directional,
// so running off the end and reappearing at the other one is a surprise, and on
// the backlight it is a nasty one: a single RIGHT press at 100% would drop the
// screen to its dimmest. RET is not directional - it is the one key that says
// "next" - so it is the one that wraps, which keeps every value reachable
// without arrow keys.
static int level_step(int cur, int dir, bool wrap) {
    int v = ((cur + LEVEL_STEP - 1) / LEVEL_STEP) * LEVEL_STEP + dir * LEVEL_STEP;
    if (v > 100) {
        v = wrap ? LEVEL_MIN : 100;
    } else if (v < LEVEL_MIN) {
        v = wrap ? 100 : LEVEL_MIN;
    }
    return v;
}

// Same rule for the CPU multiplier, which moves along a fixed ladder rather
// than in even steps.
static UINT clock_step(UINT cur, int dir, bool wrap) {
    static const uint8_t ladder[] = {BRD_CPU_MULT_LADDER};
    const int n = (int)sizeof(ladder);
    int i = 0;
    while (i < n && ladder[i] != cur) {
        i++;
    }
    if (i >= n) {
        i = 0;                      // not on the ladder: start from the bottom
    }
    i += dir;
    if (i < 0) {
        i = wrap ? n - 1 : 0;
    } else if (i >= n) {
        i = wrap ? 0 : n - 1;
    }
    return ladder[i];
}

// Every menu loop polls the keyboard, so this is the one place that is always
// reached once a screen has finished drawing - and therefore the right place to
// push the accumulated drawing out to the panel.
static bool menu_key_pop(uint8_t *nk, uint8_t *dn) {
    lcd_menu_flush();
    return usb_kbd_pop(nk, dn);
}
extern "C" int  lcd_get_scale_mode(void);        // lcd_st7789.cpp
extern "C" void lcd_set_scale_mode(int m);
extern "C" int  lcd_scale_mode_count(void);      // number of scaler modes to cycle

// nkey codes used for navigation. The numeric keypad doubles as arrows here:
// keypad 8 = up, keypad 2 = down (keypad Enter already maps to RETURN).
enum { NK_ESC = 0x00, NK_RET = 0x1c, NK_UP = 0x3a, NK_DOWN = 0x3d,
       NK_LEFT = 0x3b, NK_RIGHT = 0x3c, NK_KP8 = 0x43, NK_KP2 = 0x4b,
       NK_KP4 = 0x46, NK_KP6 = 0x48 };

// Treat main-row arrows and the keypad 8/2 the same for menu navigation.
static inline bool key_is_up(uint8_t nk)   { return nk == NK_UP   || nk == NK_KP8; }
static inline bool key_is_down(uint8_t nk) { return nk == NK_DOWN || nk == NK_KP2; }
static inline bool key_is_left(uint8_t nk)  { return nk == NK_LEFT  || nk == NK_KP4; }
static inline bool key_is_right(uint8_t nk) { return nk == NK_RIGHT || nk == NK_KP6; }

// RGB565 colors (same values as TFT_eSPI's TFT_* macros)
#define COL_WHITE  0xFFFF
#define COL_BLACK  0x0000
#define COL_YELLOW 0xFFE0

#define MAX_ENTRIES 24
#define NAME_LEN    64
#define VISIBLE     20               // rows below the title

static char s_names[MAX_ENTRIES][NAME_LEN];
static int  s_count;

#define ROMNAME_LEN 40             // main.cpp sizes the buffers to match
extern "C" char g_bios_file[ROMNAME_LEN];   // "" = the built-in default
extern "C" char g_font_file[ROMNAME_LEN];
static void save_settings(void);   // defined below; used by the HDD-eject reboot
extern "C" void usb_msc_request(int mode);  // usb_msc.cpp: arm the USB Mode boot flag
extern "C" bool gw_probe(void);             // gw_mode.cpp: Greaseweazle on the USB-A port
extern "C" bool screenshot_save(char *name, size_t cap);  // screenshot.cpp
extern "C" void sd_unmount(void);           // sd_tab5.cpp: leave the card idle
extern "C" bool fdd_gw_live_mount(int drv); // fdd_gw_live.cpp: the disk in the real drive
extern "C" bool fdd_gw_live_mounted(int drv);
extern "C" bool fdd_gw_live_info(int drv, int *cyls, int *spt, int *secsize);
extern "C" const char *gw_live_serial(int unit);  // gw_mode.cpp: which device it is
extern "C" int gw_live_port(int unit);            // ...and which hub socket it is in
// The full-screen modes indent their text to where the menu rows start.
#define MODE_INDENT "  "

#define USB_MODE_SD    1
#define USB_MODE_IMAGE 2

static const char *drv_label(int d) {
    return d == 0 ? "FDD1" : d == 1 ? "FDD2" : "HDD ";
}

static const char *drv_current(int d) {
    if (d == 2) return np2cfg.sasihdd[0][0] ? (const char *)np2cfg.sasihdd[0] : "(empty)";
    // A live drive has no file behind it, so np2cfg.fddfile[] is empty and the
    // row would say "(empty)" however well the mount had gone. Ask the backend
    // instead.
    if (fdd_gw_live_mounted(d)) {
        static char live[2][64];
        const char *sn = (d >= 0 && d < 2) ? gw_live_serial(d) : "";
        if (d < 0 || d > 1) {
            return "(GreaseWeazle - the disk in the drive)";
        }
        snprintf(live[d], sizeof(live[0]), "(GreaseWeazle on hub port %d - %s)",
                 gw_live_port(d), sn[0] ? sn : "no serial");
        return live[d];
    }
    return np2cfg.fddfile[d][0] ? (const char *)np2cfg.fddfile[d] : "(empty)";
}

static int ext_match(const char *name, int drive) {
    const char *dot = strrchr(name, '.');
    if (!dot) return 0;
    char e[8]; int i = 0;
    for (const char *p = dot + 1; *p && i < 7; p++) e[i++] = tolower((uint8_t)*p);
    e[i] = 0;
    if (drive == 2)
        return !strcmp(e, "hdi") || !strcmp(e, "nhd") || !strcmp(e, "thd") || !strcmp(e, "vhd");
    return !strcmp(e, "nfd") || !strcmp(e, "d88") || !strcmp(e, "fdi") || !strcmp(e, "fdd") || !strcmp(e, "hdm");
}

// Rows 0..10 are all written below, so this repaints in place: no clear, no
// black frame between the old screen and the new one.
// Shown on the screenshot row once one has been taken.
static char s_last_shot[24] = {0};

static void draw_drives(int sel) {
    for (int i = 0; i < 3; i++) {
        char line[96];
        snprintf(line, sizeof(line), "%s %s : %.46s", i == sel ? ">" : " ",
                 drv_label(i), drv_current(i));
        lcd_menu_line(2 + i, line, i == sel ? COL_BLACK : COL_WHITE,
                      i == sel ? COL_YELLOW : COL_BLACK);
    }
    {
        char line[64];
        snprintf(line, sizeof(line), "%s Create New Disk", sel == 3 ? ">" : " ");
        lcd_menu_line(5, line, sel == 3 ? COL_BLACK : COL_WHITE,
                      sel == 3 ? COL_YELLOW : COL_BLACK);
        snprintf(line, sizeof(line), "%s CPU clock: x%u%s",
                 sel == 4 ? ">" : " ", (unsigned)np2cfg.multiple,
                 sel == 4 ? "   (LEFT/RIGHT or RET)" : "");
        lcd_menu_line(6, line, sel == 4 ? COL_BLACK : COL_WHITE,
                      sel == 4 ? COL_YELLOW : COL_BLACK);
        // No scaler row on this board: the panel shows the PC-98 640x400 screen
        // at native size, so lcd_rgb.cpp's scaler API is a one-mode stub. (And
        // no LCD SPI clock row either - this panel is driven over the parallel
        // RGB bus; only its power-on init sequence goes over SPI.)
        // The ROM in use, chosen from whatever BIOS*.ROM / FONT*.ROM sit in the SD
        // root. Both are read once at pccore_init, so a change needs a reboot.
        snprintf(line, sizeof(line), "%s BIOS: %.24s", sel == 5 ? ">" : " ",
                 g_bios_file[0] ? g_bios_file : "(built-in compatible)");
        lcd_menu_line(7, line, sel == 5 ? COL_BLACK : COL_WHITE,
                      sel == 5 ? COL_YELLOW : COL_BLACK);
        snprintf(line, sizeof(line), "%s FONT: %.24s", sel == 6 ? ">" : " ",
                 g_font_file[0] ? g_font_file : "(built-in compatible)");
        lcd_menu_line(8, line, sel == 6 ? COL_BLACK : COL_WHITE,
                      sel == 6 ? COL_YELLOW : COL_BLACK);
        snprintf(line, sizeof(line), "%s Backlight: %d%%%s", sel == 7 ? ">" : " ",
                 g_backlight_pct, sel == 7 ? "   (LEFT/RIGHT or RET)" : "");
        lcd_menu_line(9, line, sel == 7 ? COL_BLACK : COL_WHITE,
                      sel == 7 ? COL_YELLOW : COL_BLACK);
        snprintf(line, sizeof(line), "%s Volume: %d%%%s", sel == 8 ? ">" : " ",
                 audio_get_volume(), sel == 8 ? "   (LEFT/RIGHT or RET)" : "");
        lcd_menu_line(10, line, sel == 8 ? COL_BLACK : COL_WHITE,
                      sel == 8 ? COL_YELLOW : COL_BLACK);
        // The last file written, so the row itself is the confirmation - a
        // screenshot needs no screen of its own to report one line.
        snprintf(line, sizeof(line), "%s Screenshot: %s", sel == 9 ? ">" : " ",
                 s_last_shot[0] ? s_last_shot : "save the screen as PNG");
        lcd_menu_line(11, line, sel == 9 ? COL_BLACK : COL_WHITE,
                      sel == 9 ? COL_YELLOW : COL_BLACK);
        // Reboot into the SD card reader (usb_msc.cpp). One-shot: the flag is
        // consumed at boot, so replugging USB comes back as the emulator.
        snprintf(line, sizeof(line), "%s SD Card Reader Mode", sel == 10 ? ">" : " ");
        lcd_menu_line(12, line, sel == 10 ? COL_BLACK : COL_WHITE,
                      sel == 10 ? COL_YELLOW : COL_BLACK);
        // One level down from the row above: instead of the card, the PC gets
        // the DOS volume inside the mounted HDD image.
        snprintf(line, sizeof(line), "%s Disk Image Reader Mode", sel == 11 ? ">" : " ");
        lcd_menu_line(13, line, sel == 11 ? COL_BLACK : COL_WHITE,
                      sel == 11 ? COL_YELLOW : COL_BLACK);
        // Last, where an action that throws the machine away belongs - not in
        // the middle of the settings, a keypress away from the volume.
        snprintf(line, sizeof(line), "%s RESET (save & reboot)", sel == 12 ? ">" : " ");
        lcd_menu_line(14, line, sel == 12 ? COL_BLACK : COL_WHITE,
                      sel == 12 ? COL_YELLOW : COL_BLACK);
    }
}

// Collect matching SD images into s_names. Returns entry count.
static int scan_images(int drive) {
    s_count = 0;
    // Inject the currently-mounted image first (belt-and-suspenders; de-duped
    // below) so a drive never shows "No Image" while an image is mounted.
    const char *mnt = (drive == 2) ? (const char *)np2cfg.sasihdd[0]
                                   : (const char *)np2cfg.fddfile[drive];
    if (mnt && mnt[0]) {
        const char *mb = strrchr(mnt, '/'); mb = mb ? mb + 1 : mnt;
        if (ext_match(mb, drive)) {
            snprintf(s_names[s_count], NAME_LEN, "/%s", mb);
            s_count++;
        }
    }
    // List the SD root with POSIX readdir. Unlike Arduino SD's openNextFile()
    // (which open()s each entry and stops at the first file already held open by
    // the emulator, e.g. FONT.ROM/HDD.NHD), readdir only reads directory entries,
    // so every image is listed whether or not it is currently mounted.
    DIR *dir = opendir("/sd");        // Arduino SD VFS mountpoint (see dosio_sd.cpp)
    if (!dir) return s_count;
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && s_count < MAX_ENTRIES) {
        const char *base = ent->d_name;
        if (!ext_match(base, drive)) continue;
        int dup = 0;                        // s_names entries are "/BASE"
        for (int i = 0; i < s_count; i++)
            if (!strcmp(s_names[i] + 1, base)) { dup = 1; break; }
        if (!dup) {
            snprintf(s_names[s_count], NAME_LEN, "/%s", base);
            s_count++;
        }
    }
    closedir(dir);
    return s_count;
}

// ROM chooser. The SD root is listed for BIOS*.ROM / FONT*.ROM (case-insensitive),
// so several dumps can sit on the card at once - BIOS.ROM and BIOS_9821.ROM
// both match - and the choice is a filename rather than a flag. The compatible
// BIOS is not among them: it is built into the firmware and is what runs when
// nothing here is chosen.
static int scan_roms(const char *prefix) {
    s_count = 0;
    DIR *dir = opendir("/sd");
    if (!dir) return 0;
    const size_t pl = strlen(prefix);
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && s_count < MAX_ENTRIES) {
        const char *b = ent->d_name;
        if (strncasecmp(b, prefix, pl) != 0) continue;
        const char *dot = strrchr(b, '.');
        if (!dot || strcasecmp(dot, ".rom") != 0) continue;
        snprintf(s_names[s_count], NAME_LEN, "/%s", b);
        s_count++;
    }
    closedir(dir);
    return s_count;
}

// which: 0 = BIOS, 1 = FONT. Writes the chosen name into g_bios_file/g_font_file.
static void browse_rom(int which) {
    const char *prefix = which ? "FONT" : "BIOS";
    char *dst = which ? g_font_file : g_bios_file;
    scan_roms(prefix);
    lcd_menu_clear();
    // One row above whatever is on the card: the ROM built into the firmware.
    // Without it a choice could be made but never taken back, since the list
    // is otherwise SD files only.
    const int total = s_count + 1;
    int sel = 0, top = 0;
    for (;;) {
        char hdr[64];
        snprintf(hdr, sizeof(hdr), "  select %s ROM (needs RESET)", prefix);
        lcd_menu_line(0, hdr, COL_YELLOW, COL_BLACK);
        for (int r = 0; r < VISIBLE; r++) {
            int idx = top + r;
            if (idx >= total) break;
            char line[80];
            snprintf(line, sizeof(line), "%s %.38s", idx == sel ? ">" : " ",
                     idx == 0 ? "(built-in compatible ROM)" : s_names[idx - 1]);
            lcd_menu_line(2 + r, line, idx == sel ? COL_BLACK : COL_WHITE,
                          idx == sel ? COL_YELLOW : COL_BLACK);
        }
        if (total - top < VISIBLE) {
            lcd_menu_blank_rows(2 + (total - top), 2 + VISIBLE - 1);
        }
        uint8_t nk, dn;
        for (;;) {
            if (menu_key_pop(&nk, &dn) && dn) break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (nk == NK_ESC) return;
        if (key_is_up(nk)   && sel > 0)            sel--;
        if (key_is_down(nk) && sel < total - 1)  sel++;
        if (sel < top) top = sel;
        if (sel >= top + VISIBLE) top = sel - VISIBLE + 1;
        if (nk == NK_RET) {
            if (sel == 0) {
                dst[0] = 0;            // empty = the firmware's own ROM
                ets_printf("menu: %s ROM <- built-in\n", prefix);
            } else {
                snprintf(dst, ROMNAME_LEN, "%s", s_names[sel - 1]);
                ets_printf("menu: %s ROM <- %s\n", prefix, dst);
            }
            return;
        }
    }
}

// Only a floppy drive can hold a physical disk, and only when a Greaseweazle
// is actually plugged in - offering the row otherwise would be a dead end.
static int live_rows(int drive) {
    return (drive < 2) ? 1 : 0;
}

static void draw_files(int drive, int sel, int top) {
    // The row at index s_count is the drive's eject entry. FDD ejects live; HDD
    // (the running system disk) ejects by persisting the empty state and
    // rebooting, so the machine comes back up in N88-BASIC / a floppy instead of
    // having the booted disk yanked out from under the OS.
    const char *ejlabel = (drive == 2) ? "(eject HDD & reboot)" : "(eject / empty)";
    // Floppy drives get one more row than there are files: the disk physically
    // in the drive attached to the Greaseweazle. It belongs in this list rather
    // than on a menu row of its own, because from the machine's point of view
    // it is simply another thing FDD1 can have in it.
    const int extra = live_rows(drive);
    for (int r = 0; r < VISIBLE; r++) {
        int idx = top + r;
        if (idx > s_count + extra) break;
        char line[80];
        const char *nm = (idx == s_count) ? ejlabel
                       : (idx == s_count + 1) ? "(GreaseWeazle - the disk in the drive)"
                       : s_names[idx];
        snprintf(line, sizeof(line), "%s %.48s", idx == sel ? ">" : " ", nm);
        lcd_menu_line(2 + r, line, idx == sel ? COL_BLACK : COL_WHITE,
                      idx == sel ? COL_YELLOW : COL_BLACK);
    }
    // s_count entries, the eject row, and on a floppy drive the live row;
    // blank whatever is left below them.
    const int used = (s_count + 1 + extra) - top;
    if (used < VISIBLE) {
        lcd_menu_blank_rows(2 + (used < 0 ? 0 : used), 2 + VISIBLE - 1);
    }
}

static void apply_image(int drive, const char *path) {
    if (drive == 2) {
        diskdrv_setsxsi(0x00, (const OEMCHAR *)path);
        diskdrv_hddbind();
    } else if (path) {
        diskdrv_readyfddex((REG8)drive, (const OEMCHAR *)path, FTYPE_NONE, 0);
    } else {
        diskdrv_setfddex((REG8)drive, NULL, FTYPE_NONE, 0);   // eject
    }
    ets_printf("menu: %s <- %s\n", drv_label(drive), path ? path : "(eject)");
}

// File browser for one drive; returns when an image was applied or ESC.
static void browse(int drive) {
    scan_images(drive);
    const int extra = live_rows(drive);
    int total = s_count + 1 + extra;               // +1 eject, +1 the physical drive
    lcd_menu_clear();                             // leave the drive list cleanly (no overlap)
    if (total == 0) {
        lcd_menu_line(2, "  no images on SD", COL_WHITE, COL_BLACK);
        vTaskDelay(pdMS_TO_TICKS(1200));
        return;
    }
    int sel = 0, top = 0;
    draw_files(drive, sel, top);
    for (;;) {
        uint8_t nk, dn;
        if (!menu_key_pop(&nk, &dn)) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (!dn) continue;
        if (nk == NK_ESC) return;
        if (key_is_up(nk)   && sel > 0)          sel--;
        if (key_is_down(nk) && sel < total - 1)  sel++;
        if (sel < top) top = sel;
        if (sel >= top + VISIBLE) top = sel - VISIBLE + 1;
        if (nk == NK_RET) {
            if (sel == s_count + 1 && extra) {    // the disk in the real drive
                lcd_menu_clear();
                lcd_menu_line(0, "  GreaseWeazle", COL_YELLOW, COL_BLACK);
                lcd_menu_line(2, "  Reading the disk in the drive. It stays in the",
                              COL_WHITE, COL_BLACK);
                lcd_menu_line(3, "  drive while the machine runs - nothing is written",
                              COL_WHITE, COL_BLACK);
                lcd_menu_line(4, "  to the card, and nothing is written back to it.",
                              COL_WHITE, COL_BLACK);
                lcd_menu_flush();
                const bool ok = fdd_gw_live_mount(drive);
                char msg[80];
                int cyls = 0, spt = 0, ss = 0;
                if (ok && fdd_gw_live_info(drive, &cyls, &spt, &ss)) {
                    snprintf(msg, sizeof(msg),
                             "  Ready: %d cyl x 2 head x %d sect x %d bytes",
                             cyls, spt, ss);
                } else {
                    snprintf(msg, sizeof(msg), "  Failed - see the serial log.");
                }
                lcd_menu_line(6, msg, COL_WHITE, COL_BLACK);
                if (ok) {
                    char who[80];
                    const char *sn = gw_live_serial(drive);
                    snprintf(who, sizeof(who), "  GreaseWeazle on hub port %d, serial %s",
                             gw_live_port(drive), sn[0] ? sn : "(none)");
                    lcd_menu_line(7, who, COL_YELLOW, COL_BLACK);
                }
                lcd_menu_flush();
                vTaskDelay(pdMS_TO_TICKS(ok ? 3000 : 2500));
                return;
            }
            if (sel == s_count) {                 // eject entry
                if (drive == 2) {
                    // Eject the running HDD: persist the now-empty HDD and reboot,
                    // so the machine comes up in N88-BASIC / a floppy instead of
                    // live-unmounting the disk the OS is booted from.
                    np2cfg.sasihdd[0][0] = '\0';
                    lcd_menu_clear();
                    lcd_menu_line(2, "  HDD ejected - rebooting...", COL_WHITE, COL_BLACK);
                    // Same reason as the reboot out of USB Mode: the framebuffer
                    // lives in PSRAM, so without pushing the cache out the clear
                    // and the text never reach it, and the panel keeps whatever
                    // the cache happened to evict - half the old menu, left as
                    // character debris.
                    lcd_menu_flush();
                    save_settings();
                    vTaskDelay(pdMS_TO_TICKS(800));
                    // And hand the card back, so the boot that follows can
                    // initialise it from scratch rather than find it busy.
                    sd_unmount();
                    panel_tab5_blank_early();
                    esp_restart();
                }
                apply_image(drive, NULL);         // FDD: live eject
            } else {
                apply_image(drive, s_names[sel]);
            }
            vTaskDelay(pdMS_TO_TICKS(600));
            return;
        }
        draw_files(drive, sel, top);
    }
}

// nkey -> ASCII for filename entry (uppercase letters, digits, '-')
static char nkey_ascii(uint8_t nk) {
    if (nk >= 0x01 && nk <= 0x09) return (char)('1' + nk - 1);
    if (nk == 0x0a) return '0';
    if (nk == 0x0b) return '-';
    static const char letters[0x30] = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,                  // 0x00-0x0f
        'q','w','e','r','t','y','u','i','o','p',0,0,0,'a','s','d', // 0x10-0x1f
        'f','g','h','j','k','l',0,0,0,'z','x','c','v','b','n','m'  // 0x20-0x2f
    };
    if (nk < 0x30 && letters[nk]) return (char)(letters[nk] - 'a' + 'A');
    switch (nk) {                        // numeric keypad digits KP0..KP9
        case 0x4e: return '0'; case 0x4a: return '1'; case 0x4b: return '2';
        case 0x4c: return '3'; case 0x46: return '4'; case 0x47: return '5';
        case 0x48: return '6'; case 0x42: return '7'; case 0x43: return '8';
        case 0x44: return '9';
    }
    return 0;
}

static uint8_t wait_key(void) {
    uint8_t nk, dn;
    for (;;) {
        if (!menu_key_pop(&nk, &dn)) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (dn) return nk;
    }
}

// Create a blank .NHD image, written in small chunks with a progress line.
//
// np2kai's newdisk_nhd_ex() cannot be used for this: writehddiplex2() sizes its
// work buffer from the image size and asks malloc() for 8MB for anything larger
// than 8MB. That allocation cannot succeed while the emulator holds most of the
// PSRAM, and it then returned FAILURE and file_delete()d the image it had just
// created - "Create New Disk" said "done" and left nothing on the card. The
// on-disk result is the same as np2kai's blank=1 (T98 NHD header + zeros), but
// written 16KB at a time, so progress can be shown and ESC can cancel.
static bool create_nhd(const char *path, UINT mb, bool *cancelled) {
    const UINT32 C = (UINT32)mb * 15;            // np2kai hddsize2CHS(), <=4351MB
    const UINT16 H = 8, S = 17, SS = 512;
    const uint64_t total = (uint64_t)C * H * S * SS;

    NHDHDR nhd;
    ZeroMemory(&nhd, sizeof(nhd));
    CopyMemory(nhd.sig, sig_nhd, 15);
    STOREINTELDWORD(nhd.headersize, sizeof(nhd));
    STOREINTELDWORD(nhd.cylinders, C);
    STOREINTELWORD(nhd.surfaces, H);
    STOREINTELWORD(nhd.sectors, S);
    STOREINTELWORD(nhd.sectorsize, SS);

    // Internal DMA-capable RAM when some is free (the SD driver writes straight
    // out of it); otherwise ordinary PSRAM, which dosio_sd bounces in 2KB pieces.
    const UINT chunk = 16 * 1024;
    uint8_t *work = (uint8_t *)heap_caps_malloc(chunk, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!work) work = (uint8_t *)malloc(chunk);
    if (!work) return false;
    memset(work, 0, chunk);

    FILEH fh = file_create((const OEMCHAR *)path);
    if (fh == FILEH_INVALID) { free(work); return false; }

    bool ok = (file_write(fh, &nhd, sizeof(nhd)) == sizeof(nhd));
    uint64_t left = total;
    int lastpct = -1;
    while (ok && left) {
        UINT n = (UINT)((left < chunk) ? left : chunk);
        if (file_write(fh, work, n) != n) { ok = false; break; }
        left -= n;
        int pct = (int)(((total - left) * 100) / total);
        if (pct != lastpct) {
            char line[48];
            snprintf(line, sizeof(line), "  creating... %d%%  (ESC: cancel)", pct);
            lcd_menu_line(4, line, COL_WHITE, COL_BLACK);
            lastpct = pct;
        }
        uint8_t nk, dn;
        while (menu_key_pop(&nk, &dn))
            if (dn && nk == NK_ESC) { *cancelled = true; ok = false; }
    }
    file_close(fh);
    free(work);
    if (!ok) file_delete((const OEMCHAR *)path);      // no half-written image
    return ok;
}

// Create a blank FD/HDD image on the SD card (name typed on USB keyboard).
static void newdisk_flow(void) {
    static const struct { const char *label; int mb; } types[] = {
        { "FD 1.44MB (2HD .HDM)", 0 },
        { "HDD 20MB (.NHD)", 20 },
        { "HDD 40MB (.NHD)", 40 },
        { "HDD 80MB (.NHD)", 80 },
    };
    int sel = 0;
    lcd_menu_clear();
    for (;;) {
        lcd_menu_line(1, "  New blank disk: type", COL_WHITE, COL_BLACK);
        for (int i = 0; i < 4; i++) {
            char line[64];
            snprintf(line, sizeof(line), "%s %s", i == sel ? ">" : " ", types[i].label);
            lcd_menu_line(3 + i, line, i == sel ? COL_BLACK : COL_WHITE,
                          i == sel ? COL_YELLOW : COL_BLACK);
        }
        uint8_t nk = wait_key();
        if (nk == NK_ESC) return;
        if (key_is_up(nk)   && sel > 0) sel--;
        if (key_is_down(nk) && sel < 3) sel++;
        if (nk == NK_RET) break;
    }
    int mb = types[sel].mb;

    // base name entry: up to 8 chars (8.3 style), RET to confirm
    char name[9]; int len = 0; name[0] = 0;
    lcd_menu_clear();
    for (;;) {
        lcd_menu_line(1, "  File name (A-Z 0-9 -)", COL_WHITE, COL_BLACK);
        char line[32];
        snprintf(line, sizeof(line), "  %s_", name);
        lcd_menu_line(3, line, COL_WHITE, COL_BLACK);
        lcd_menu_line(5, "  RET:create BS:del ESC:cancel", COL_WHITE, COL_BLACK);
        uint8_t nk = wait_key();
        if (nk == NK_ESC) return;
        if (nk == NK_RET) { if (len > 0) break; continue; }
        if (nk == 0x0e) { if (len > 0) name[--len] = 0; continue; }  // Backspace
        char c = nkey_ascii(nk);
        if (c && len < 8) { name[len++] = c; name[len] = 0; }
    }

    char path[24];
    snprintf(path, sizeof(path), "/%s.%s", name, mb ? "NHD" : "HDM");
    lcd_menu_clear();
    lcd_menu_line(1, "  New blank disk:", COL_WHITE, COL_BLACK);
    lcd_menu_line(2, path, COL_WHITE, COL_BLACK);

    // stat() rather than SD.exists(): the card is mounted on the VFS at /sd,
    // so the standard call reaches it and no Arduino layer is needed.
    struct stat st_;
    char full_[160];
    snprintf(full_, sizeof(full_), "/sd%s", path);
    if (stat(full_, &st_) == 0) {
        lcd_menu_line(4, "  exists! aborted", COL_WHITE, COL_BLACK);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    lcd_menu_line(4, "  creating...", COL_WHITE, COL_BLACK);
    bool ok, cancelled = false;
    if (mb == 0) {
        newdisk_144mb_fdd((const OEMCHAR *)path);
        ok = (file_attr((const OEMCHAR *)path) >= 0);   // it reports nothing itself
    } else {
        ok = create_nhd(path, (UINT)mb, &cancelled);
    }
    lcd_menu_line(4, cancelled ? "  cancelled"
                    : ok       ? "  done (format in DOS)"
                               : "  FAILED (card full or write error)",
                  COL_WHITE, COL_BLACK);
    vTaskDelay(pdMS_TO_TICKS(1500));
}

// Persist the 5 menu items — CPU multiple, scaler mode, and the FDD1/FDD2/HDD
// mounts — called on menu ESC exit and on RESET. main.cpp reloads these at boot
// and restores the disks. NVS commit only writes flash when a value actually
// changed, so this is wear-safe.
static void save_settings(void) {
    nvs_handle_t nh;
    if (nvs_open("pc98", NVS_READWRITE, &nh) == ESP_OK) {
        nvs_set_u8(nh, "multiple", (uint8_t)np2cfg.multiple);
        nvs_set_u8(nh, "scaler", (uint8_t)lcd_get_scale_mode());
        nvs_set_u8(nh, "backlight", (uint8_t)g_backlight_pct);
        nvs_set_u8(nh, "volume", (uint8_t)audio_get_volume());
        nvs_set_str(nh, "biosfile", g_bios_file);
        nvs_set_str(nh, "fontfile", g_font_file);
        // Current mounts (empty drive => empty string; restored as empty at boot).
        nvs_set_str(nh, "fdd0", (const char *)np2cfg.fddfile[0]);
        nvs_set_str(nh, "fdd1", (const char *)np2cfg.fddfile[1]);
        nvs_set_str(nh, "hdd",  (const char *)np2cfg.sasihdd[0]);
        nvs_commit(nh);
        nvs_close(nh);
    }
}

extern "C" void menu_disk_run(void) {
    int sel = 0;
    lcd_menu_clear();
    draw_drives(sel);
    for (;;) {
        uint8_t nk, dn;
        if (!menu_key_pop(&nk, &dn)) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (!dn) continue;
        if (nk == NK_ESC) break;
        if (key_is_up(nk)   && sel > 0) sel--;
        if (key_is_down(nk) && sel < 12) sel++;
        // Left/right adjust the rows that hold a value rather than doing
        // something; everywhere else they are ignored.
        {
            const int dir = key_is_right(nk) ? 1 : (key_is_left(nk) ? -1 : 0);
            if (dir) {
                if (sel == 7) {
                    g_backlight_pct = level_step(g_backlight_pct, dir, false);
                    tab5_backlight_set(g_backlight_pct);
                } else if (sel == 8) {
                    audio_set_volume(level_step(audio_get_volume(), dir, false));
                } else if (sel == 4) {
                    const UINT mult = clock_step(np2cfg.multiple, dir, false);
                    np2cfg.multiple = mult;       // display follows immediately
                    g_speed_req = (int)mult;      // emulator loop applies it
                }
            }
        }
        if (nk == NK_RET) {
            if (sel == 3) {                     // create blank disk image
                newdisk_flow();
                lcd_menu_clear();   // the submenu owned the screen
            } else if (sel == 4) {              // CPU clock: next, wrapping
                const UINT mult = clock_step(np2cfg.multiple, 1, true);
                np2cfg.multiple = mult;         // display follows immediately
                g_speed_req = (int)mult;        // emulator loop applies it
            } else if (sel == 5) {              // BIOS ROM: pick a file from the SD
                browse_rom(0);
                lcd_menu_clear();   // the submenu owned the screen
            } else if (sel == 6) {              // FONT ROM: pick a file from the SD
                browse_rom(1);
                lcd_menu_clear();   // the submenu owned the screen
            } else if (sel == 7) {              // Backlight: next, wrapping
                g_backlight_pct = level_step(g_backlight_pct, 1, true);
                tab5_backlight_set(g_backlight_pct);
            } else if (sel == 8) {              // Volume: next, wrapping
                audio_set_volume(level_step(audio_get_volume(), 1, true));
            } else if (sel == 9) {              // Screenshot
                if (!screenshot_save(s_last_shot, sizeof(s_last_shot))) {
                    snprintf(s_last_shot, sizeof(s_last_shot), "FAILED");
                }
            } else if (sel == 10 || sel == 11) {   // USB Mode: arm the flag, reboot
                const int m = (sel == 10) ? USB_MODE_SD : USB_MODE_IMAGE;
                save_settings();
                usb_msc_request(m);
                lcd_menu_clear();
                lcd_menu_line(0, (m == USB_MODE_IMAGE) ? MODE_INDENT "Disk Image Reader Mode"
                                                       : MODE_INDENT "SD Card Reader Mode",
                              COL_YELLOW, COL_BLACK);
                lcd_menu_line(2, MODE_INDENT "Restarting...", COL_WHITE, COL_BLACK);
                // Drawing normally reaches the panel from menu_key_pop(), and
                // this is the one screen that never waits for a key - it
                // reboots. Without this the clear and the text stay in cache,
                // only whatever the cache happened to evict gets to PSRAM, and
                // what is left on the panel is half the old menu.
                lcd_menu_flush();
                vTaskDelay(pdMS_TO_TICKS(800));
                // Hand the card back before the reboot. The mode this restarts
                // into initialises the card from scratch, and one left busy by
                // the emulator will not answer - which is what made entering
                // these modes work only some of the time.
                sd_unmount();
                panel_tab5_blank_early();
                esp_restart();
            } else if (sel == 12) {             // RESET: persist the settings, reboot
                save_settings();
                sd_unmount();                   // same reason as USB Mode above
                panel_tab5_blank_early();
                esp_restart();
            } else {
                browse(sel);                    // sel 0/1/2 = FDD1/FDD2/HDD
                lcd_menu_clear();   // the submenu owned the screen
            }
        }
        draw_drives(sel);
    }
    save_settings();                        // ESC exit: persist multiple + scaler
}
