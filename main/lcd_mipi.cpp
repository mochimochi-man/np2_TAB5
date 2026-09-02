// Display backend for the M5Stack Tab5: 2-lane MIPI-DSI, 720(H) x 1280(V)
// native portrait, RGB565. The panel controller is ILI9881C on board revision
// 1 and ST7123 on revision 2 — the BSP probes for it, so nothing here depends
// on which one is fitted.
//
// Same extern "C" interface as the ESP32-S3 backends (lcd_rgb.cpp /
// lcd_st7789.cpp) so main.cpp and menu_disk.cpp need no changes. What differs:
//
//   - The panel is driven by esp_lcd's DPI driver, which — like the S3's RGB
//     driver — hands out a frame buffer that hardware scans out continuously.
//     So this is a blit into memory, not a transfer, and the panel keeps
//     showing whatever was left there. That is what makes the per-tile diff
//     below safe.
//   - The panel is portrait and the PC-98 screen is landscape, so the blit
//     rotates 90 degrees: PC-98 x runs along the panel's 1280 axis, PC-98 y
//     along its 720 axis.
//   - Two placements, switchable at runtime (see board_pins.h): 1.8x filling
//     the height, or 1:1 centred.

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "driver/ppa.h"

// The panel itself is brought up in panel_tab5.cpp, which keeps the BSP
// headers to itself — they pull in esp_vfs_fat.h, whose ff.h typedefs TCHAR as
// char while np2kai's compiler_base.h typedefs it as wchar_t, and no
// translation unit can see both. This file needs np2kai, for the CGROM.
extern "C" bool panel_tab5_init(void);
extern "C" uint16_t *panel_tab5_framebuffer(void);

extern "C" {
#include <compiler.h>  // np2kai types
#include <i286c/cpumem.h>
#include <font/font.h>  // fontrom: the CGROM loaded from FONT.ROM
extern UINT8 mem[];     // np2kai main memory (cpumem.c)
}

#include "board_pins.h"

extern "C" int ets_printf(const char *fmt, ...);

#define PC98_W BRD_PC98_W
#define PC98_H BRD_PC98_H
#define PAN_W  BRD_LCD_H_RES  // 720
#define PAN_H  BRD_LCD_V_RES  // 1280
#define SRC_W  PC98_W

// Which way up the picture sits on the glass, as one switch.
//
// Three things have to agree or the display comes apart: the PPA's rotation of
// the emulated screen, the s_map_* tables the menu text is painted through, and
// the same tables again for turning a finger position back into a PC-98
// coordinate. Flipping both axes of the tables is a 180-degree rotation, and
// swapping the PPA between 90 and 270 is the same 180 degrees, so they are tied
// together here rather than left as three independent constants to get wrong.
//
// 1 puts the picture the right way up for the way the case is marked.
#define ROTATE_180 1

#if ROTATE_180
#define FLIP_V 0
#define FLIP_H 1
#define PPA_ROTATION PPA_SRM_ROTATION_ANGLE_90
#else
#define FLIP_V 1
#define FLIP_H 0
#define PPA_ROTATION PPA_SRM_ROTATION_ANGLE_270
#endif

// 16, not the 32 the RGB fork uses: 1280 divides by 32 but 720 does NOT
// (22.5), so a 32 tile silently covered 704 of the 720 columns. 16 divides
// both exactly (45 x 80 tiles).
//
// The tile is what keeps the strided source reads inside the cache: a whole
// tile is drawn from at most 16 source rows, and those stay resident for the
// tile's 16 output rows. Walking the source the other way misses on every
// pixel — that cost the RGB fork a 7.8x slowdown before it was tiled.
#define TILE 16
#define TILES_X (PAN_W / TILE)
#define TILES_Y (PAN_H / TILE)

static uint16_t *s_fb = nullptr;  // the DPI frame buffer (PSRAM)

// panel axis -> PC-98 axis, precomputed for the active scale mode.
// -1 means "outside the PC-98 screen", i.e. letterbox.
static int16_t s_map_y[PAN_W];  // panel_x -> PC-98 y
static int16_t s_map_x[PAN_H];  // panel_y -> PC-98 x

static int s_scale_mode = BRD_SCALE_DEFAULT;
static bool s_force_full = true;

// Per-tile hash of what was last written, so an unchanged tile costs no PSRAM
// writes at all. On a static screen that removes almost the whole blit.

static SemaphoreHandle_t s_blit_req = nullptr;
static const uint8_t *volatile s_blit_src = nullptr;
static volatile int s_blit_busy = 0;

// ---- scale mapping --------------------------------------------------------
static void build_maps(void) {
    for (int i = 0; i < PAN_W; i++) {
        s_map_y[i] = -1;
    }
    for (int i = 0; i < PAN_H; i++) {
        s_map_x[i] = -1;
    }

    if (s_scale_mode == BRD_SCALE_DOT) {
        // 1:1, centred on both axes.
        const int xoff = (PAN_W - PC98_H) / 2;  // (720-400)/2 = 160
        const int yoff = (PAN_H - PC98_W) / 2;  // (1280-640)/2 = 320
        for (int py = 0; py < PC98_H; py++) {
            s_map_y[xoff + py] = (int16_t)(FLIP_V ? (PC98_H - 1 - py) : py);
        }
        for (int px = 0; px < PC98_W; px++) {
            s_map_x[yoff + px] = (int16_t)(FLIP_H ? (PC98_W - 1 - px) : px);
        }
    } else {
        // 1.8x: 400 -> 720 exactly (the whole short axis), 640 -> 1152.
        const int outw = PC98_H * BRD_FIT_NUM / BRD_FIT_DEN;  // 720
        const int outh = PC98_W * BRD_FIT_NUM / BRD_FIT_DEN;  // 1152
        const int xoff = (PAN_W - outw) / 2;                  // 0
        const int yoff = (PAN_H - outh) / 2;                  // 64
        for (int i = 0; i < outw; i++) {
            int py = i * BRD_FIT_DEN / BRD_FIT_NUM;
            if (py >= PC98_H) {
                py = PC98_H - 1;
            }
            s_map_y[xoff + i] = (int16_t)(FLIP_V ? (PC98_H - 1 - py) : py);
        }
        for (int i = 0; i < outh; i++) {
            int px = i * BRD_FIT_DEN / BRD_FIT_NUM;
            if (px >= PC98_W) {
                px = PC98_W - 1;
            }
            s_map_x[yoff + i] = (int16_t)(FLIP_H ? (PC98_W - 1 - px) : px);
        }
    }
    s_force_full = true;
}

// ---- blit -----------------------------------------------------------------
// The panel's DMA reads the frame buffer straight out of PSRAM while the CPU
// writes to it through the L2 cache, so anything written has to be flushed
// before the scanout can see it. Without this the panel is lit, the driver is
// fetching, the emulator is running — and the screen stays black, because the
// DMA keeps reading the zeroed memory the cache is hiding. IDF's own DPI
// driver does exactly this in esp_lcd_panel_dpi.c whenever it touches a frame
// buffer. Synced a tile row-band at a time (32 panel rows, contiguous) rather
// than the whole 1.8MB, so an unchanged screen costs nothing.
static inline void fb_sync_all(void) {
    esp_cache_msync(s_fb, (size_t)PAN_W * PAN_H * sizeof(uint16_t),
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static inline void fb_sync_band(int y0) {
    void *p = s_fb + (size_t)y0 * PAN_W;
    const size_t n = (size_t)TILE * PAN_W * sizeof(uint16_t);
    esp_cache_msync(p, n, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

// The rotate-and-scale is done by the PPA, the ESP32-P4's 2D pixel engine.
//
// The CPU version that used to live here cost 62ms for a full frame - more than
// the emulated CPU and the screen composite put together, and enough that the
// emulator throttled its own frame rate waiting for it (main.cpp drops to
// rendering every 4th frame when it falls behind). It is 921,600 output pixels
// gathered through a rotation, so there was not much left to win by tuning it.
//
// The PPA does exactly this operation in hardware: 90 degrees counter-clockwise
// puts PC-98 x along the panel's long axis and mirrors y, which is precisely the
// mapping the tables above describe, and it does it as a DMA pass rather than a
// million strided loads.
//
// What it costs is the scale factor. The PPA expresses it as an integer plus a
// 1/16 fraction, so the 1.8x that filled the panel's short axis exactly is not
// available; 1.75 is the closest below it and leaves a 10-pixel border on that
// axis. board_pins.h carries the same ratio so the menu text and the touch
// mapping stay in step.
static ppa_client_handle_t s_ppa = nullptr;

static void do_blit(const uint8_t *fb) {
    if (!s_ppa) {
        return;
    }
    const int num = (s_scale_mode == BRD_SCALE_DOT) ? 1 : BRD_FIT_NUM;
    const int den = (s_scale_mode == BRD_SCALE_DOT) ? 1 : BRD_FIT_DEN;
    const int outw = PC98_H * num / den;   // along the panel's short axis
    const int outh = PC98_W * num / den;   // along the panel's long axis

    ppa_srm_oper_config_t op = {};
    op.in.buffer = (void *)fb;
    op.in.pic_w = SRC_W;
    op.in.pic_h = 480;                     // the surface np2kai draws into
    op.in.block_w = PC98_W;
    op.in.block_h = PC98_H;
    op.in.block_offset_x = 0;
    op.in.block_offset_y = 0;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    op.out.buffer = s_fb;
    op.out.buffer_size = (size_t)PAN_W * PAN_H * sizeof(uint16_t);
    op.out.pic_w = PAN_W;
    op.out.pic_h = PAN_H;
    op.out.block_offset_x = (PAN_W - outw) / 2;
    op.out.block_offset_y = (PAN_H - outh) / 2;
    op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    op.rotation_angle = PPA_ROTATION;
    op.scale_x = (float)num / (float)den;
    op.scale_y = (float)num / (float)den;
    op.mode = PPA_TRANS_MODE_BLOCKING;

    ppa_do_scale_rotate_mirror(s_ppa, &op);
    s_force_full = false;
}

// The PPA only writes the block; the letterbox around it keeps whatever was
// there. Painted black once, when the geometry changes.
static void clear_borders(void) {
    if (s_fb) {
        memset(s_fb, 0, (size_t)PAN_W * PAN_H * sizeof(uint16_t));
        fb_sync_all();
    }
}

static void blit_task(void *arg) {
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_blit_req, portMAX_DELAY);
        const uint8_t *src = s_blit_src;
        if (src) {
            do_blit(src);
        }
        s_blit_busy = 0;
    }
}

// Bring-up diagnostic: the panel showed noise at power-on (so the frame buffer
// is on screen) and then black (so our clear and the cache sync both work),
// which leaves only "is anything being blitted, and is it non-black?".
// Set to 0 once the picture is up.
#define LCD_BLIT_TRACE 0

extern "C" void lcd_blit(const uint8_t *fb) {
#if LCD_BLIT_TRACE
    static int calls = 0, dropped = 0;
    if (s_fb && fb && s_blit_busy) {
        dropped++;
    }
    if (s_fb && fb && !s_blit_busy && (calls++ % 60) == 0) {
        // Is the emulator's own framebuffer actually non-black?
        const uint16_t *p = (const uint16_t *)fb;
        int nz = 0;
        for (int y = 0; y < PC98_H && nz < 4; y += 8) {
            for (int x = 0; x < PC98_W; x += 8) {
                if (p[(size_t)y * SRC_W + x]) {
                    nz++;
                    break;
                }
            }
        }
        // The boot screen is "text VRAM rendered with the CGROM", so an all
        // black frame buffer is either an empty text VRAM (the BIOS wrote
        // nothing) or an empty font (FONT.ROM never loaded). Report both.
        int tvram = 0;
        for (int i = 0; i < 80 * 25 * 2; i += 2) {
            const uint8_t ch = mem[0xA0000 + i];
            if (ch > 0x20 && ch < 0x7f) {
                tvram++;
            }
        }
        int glyph = 0;
        for (int i = 0; i < 16; i++) {
            if (fontrom[0x80000 + 'A' * 16 + i]) {
                glyph++;
            }
        }
        // And does anything actually reach the panel's own buffer?
        int fbnz = 0;
        for (int i = 0; i < PAN_W * PAN_H; i += 977) {
            if (s_fb[i]) {
                fbnz++;
            }
        }
        ets_printf("lcd_blit: n=%d src_nb=%d tvram=%d fontA=%d fb_nb=%d\n", calls, nz, tvram, glyph, fbnz);
    }
#endif
    if (!s_fb || !fb || s_blit_busy) {
        return;  // still drawing the previous frame; drop this one
    }
    s_blit_src = fb;
    s_blit_busy = 1;
    xSemaphoreGive(s_blit_req);
}

extern "C" bool lcd_blit_busy(void) {
    return s_blit_busy != 0;
}

// The frame the emulator handed over last, for screenshot.cpp. While the menu
// is up this is still the screen that was showing when it was opened: the menu
// draws into the panel buffer, not into this one, and the machine is paused.
// The geometry returned is the PC-98 screen's own, before any scaling.
extern "C" const void *lcd_last_frame(int *w, int *h, int *stride) {
    if (w) {
        *w = PC98_W;
    }
    if (h) {
        *h = PC98_H;
    }
    if (stride) {
        *stride = SRC_W;   // np2kai draws into a 640x480 surface
    }
    return (const void *)s_blit_src;
}

// ---- scale mode API -------------------------------------------------------
extern "C" int lcd_get_scale_mode(void) {
    return s_scale_mode;
}

extern "C" int lcd_scale_mode_count(void) {
    return BRD_SCALE_COUNT;
}

extern "C" const char *lcd_scale_mode_name(int m) {
    return (m == BRD_SCALE_DOT) ? "1:1 dot" : "1.8x fit";
}

extern "C" void lcd_set_scale_mode(int m) {
    if (m < 0 || m >= BRD_SCALE_COUNT || m == s_scale_mode) {
        return;
    }
    s_scale_mode = m;
    build_maps();
    if (s_fb) {
        memset(s_fb, 0, (size_t)PAN_W * PAN_H * sizeof(uint16_t));
        fb_sync_all();
    }
}

// ---- init -----------------------------------------------------------------
extern "C" bool lcd_init(void) {
    if (!panel_tab5_init()) {
        return false;
    }
    // The DPI driver owns the frame buffer and scans it out continuously, so
    // write into it directly rather than pushing bitmaps.
    s_fb = panel_tab5_framebuffer();
    if (!s_fb) {
        return false;
    }
    memset(s_fb, 0, (size_t)PAN_W * PAN_H * sizeof(uint16_t));
    fb_sync_all();

    s_blit_req = xSemaphoreCreateBinary();
    if (!s_blit_req) {
        ets_printf("lcd: blit semaphore failed\n");
        return false;
    }

    build_maps();
    {
        ppa_client_config_t pc = {};
        pc.oper_type = PPA_OPERATION_SRM;
        pc.max_pending_trans_num = 1;
        if (ppa_register_client(&pc, &s_ppa) != ESP_OK) {
            ets_printf("lcd: ppa_register_client failed\n");
            return false;
        }
    }
    clear_borders();
    xTaskCreatePinnedToCore(blit_task, "lcdblit", 4096, nullptr, 4, nullptr, 0);

    // Backlight only — NOT bsp_display_brightness_init(). bsp_display_new()
    // has already run it (along with bsp_feature_enable(BSP_FEATURE_LCD)), and
    // configuring the same LEDC channel twice fails with
    //     ledc: GPIO 22 is not usable, maybe conflict with others
    // leaving the channel dead and the panel dark — the emulator runs, the
    // chime plays, and the screen shows nothing.
    // The panel model is not fixed: the BSP probes the board revision and picks
    // ILI9881C or ST7123 accordingly (it logs which). Only the geometry is ours
    // to care about, so do not name a controller here.
    ets_printf("lcd: MIPI-DSI %dx%d ready (PC-98 %dx%d, %s)\n", PAN_W, PAN_H, PC98_W, PC98_H, (s_scale_mode == BRD_SCALE_DOT) ? "1:1" : "1.75x fit");
    return true;
}

// ---- disk-menu text UI ----------------------------------------------------
// Glyphs come from the CGROM the emulator loaded from FONT.ROM (8x16 ANK cells
// at 0x80000), the same source the S3 forks use — no second font.
//
// The menu is laid out in PC-98 coordinates (80 columns of 8 pixels, 16-pixel
// rows) and mapped onto the panel through the same rotation and scale as the
// emulator image, so it lands on the picture rather than somewhere else on the
// glass — and the 1.8x mode duplicates its pixels exactly the way the blit
// does, with no second scaling path to get subtly wrong.
#define FONT_ANK16 0x80000
#define MENU_CH_W 8
#define MENU_CH_H 16
#define MENU_COLS (PC98_W / MENU_CH_W)  // 80
#define MENU_ROWS (PC98_H / MENU_CH_H)  // 25

// The menu draws several rows in a row and only then waits for a key, so the
// cache writeback is deferred to lcd_menu_flush() instead of being paid per
// row. A row covers ~1150 panel scanlines, so its writeback range spans very
// nearly the whole frame buffer however narrow the row is - there is nothing to
// gain by syncing a smaller region, only by syncing fewer times.
static bool s_menu_dirty = false;

extern "C" void lcd_menu_clear(void) {
    if (s_fb) {
        memset(s_fb, 0, (size_t)PAN_W * PAN_H * sizeof(uint16_t));
        s_menu_dirty = true;
    }
    s_force_full = true;  // the panel no longer matches the per-tile hashes
}

// Panel pixel -> PC-98 pixel, through whichever scale mode is active. The
// touch panel needs exactly the transform the blit uses and nothing else, so it
// reads the same two tables rather than reconstructing the rotation.
extern "C" bool lcd_panel_to_pc98(int panel_x, int panel_y, int *out_x, int *out_y) {
    if (panel_x < 0 || panel_x >= PAN_W || panel_y < 0 || panel_y >= PAN_H) {
        return false;
    }
    const int px = s_map_x[panel_y];
    const int py = s_map_y[panel_x];
    if (px < 0 || py < 0) {
        return false;  // letterbox
    }
    if (out_x) {
        *out_x = px;
    }
    if (out_y) {
        *out_y = py;
    }
    return true;
}

extern "C" void lcd_menu_flush(void) {
    if (s_menu_dirty) {
        fb_sync_all();
        s_menu_dirty = false;
    }
}

extern "C" void lcd_menu_line(int row, const char *s, uint16_t fg, uint16_t bg) {
    if (!s_fb || !s || row < 0 || row >= MENU_ROWS) {
        return;
    }
    const int py0 = row * MENU_CH_H;  // PC-98 y of the row's first scanline
    int len = 0;
    while (s[len] && len < MENU_COLS) {
        len++;
    }

    // The screen is rotated, so one text row is a narrow vertical band of panel
    // columns. Find it once rather than testing all 720 columns on every one of
    // the 1280 scanlines - that inner test was costing ~921,600 iterations per
    // row, nine rows per redraw, and it is what made moving the cursor look
    // like a full repaint.
    int xa = PAN_W, xb = -1;
    for (int panel_x = 0; panel_x < PAN_W; panel_x++) {
        const int py = s_map_y[panel_x];
        if (py >= py0 && py < py0 + MENU_CH_H) {
            if (panel_x < xa) xa = panel_x;
            if (panel_x > xb) xb = panel_x;
        }
    }
    if (xb < xa) {
        return;  // this row is outside the scaled image (letterbox)
    }

    for (int panel_y = 0; panel_y < PAN_H; panel_y++) {
        const int px = s_map_x[panel_y];
        if (px < 0) {
            continue;
        }
        const int col = px / MENU_CH_W;
        const int bit = px % MENU_CH_W;
        const uint8_t code = (col < len) ? (uint8_t)s[col] : (uint8_t)' ';
        const uint8_t *glyph = fontrom + FONT_ANK16 + (unsigned)code * MENU_CH_H;

        uint16_t *drow = s_fb + (size_t)panel_y * PAN_W;
        for (int panel_x = xa; panel_x <= xb; panel_x++) {
            drow[panel_x] = (glyph[s_map_y[panel_x] - py0] & (0x80 >> bit)) ? fg : bg;
        }
    }
    s_menu_dirty = true;
    s_force_full = true;  // the emulator image must be repainted in full after
}

// Blank a range of text rows. Redrawing a menu used to start by clearing the
// whole frame buffer, which is 1.84MB - far more than fits in cache, so the
// black reached PSRAM (and the panel) before the text was written over it and
// every cursor move flashed the screen black. Rows the new screen writes need
// no clearing at all, since each row is painted edge to edge including its
// background; only the rows it does NOT write have to be blanked.
extern "C" void lcd_menu_blank_rows(int first, int last) {
    for (int row = first; row <= last; row++) {
        lcd_menu_line(row, "", 0, 0);
    }
}
