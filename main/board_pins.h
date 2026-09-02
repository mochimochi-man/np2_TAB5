// Board geometry for the M5Stack Tab5 (ESP32-P4) port of np2 espresso.
//
// Unlike the ESP32-S3 forks, this file carries almost no pin numbers: the
// vendor BSP (espressif/m5stack_tab5) owns the wiring and exposes it as BSP_*
// macros and bsp_* calls. Duplicating those here would only create a second
// source of truth that can drift. What is left is the geometry the emulator
// itself has to reason about.
//
// See BOARD.md for the port's rationale.
#pragma once

// No BSP header at all: bsp/m5stack_tab5.h drags in esp_vfs_fat.h, whose ff.h
// typedefs TCHAR as char while np2kai's compiler_base.h typedefs it as wchar_t,
// and no translation unit can see both. panel_tab5.cpp owns the BSP side; this
// header is included by files that also need np2kai, so the two panel numbers
// are written out rather than pulled from BSP_LCD_H_RES / BSP_LCD_V_RES.

// ---- Panel ----------------------------------------------------------------
// ILI9881C over 2-lane MIPI-DSI, native portrait 720(H) x 1280(V), RGB565.
// The emulator draws PC-98 640x400 landscape, so the blit rotates 90 degrees:
// PC-98 x runs down the panel's long axis, PC-98 y across its short one.
#define BRD_LCD_H_RES       720
#define BRD_LCD_V_RES       1280

// ---- PC-98 screen ---------------------------------------------------------
#define BRD_PC98_W          640
#define BRD_PC98_H          400

// Two ways to place 640x400 on the rotated 1280x720 panel, switchable at
// runtime — neither is obviously right, so both exist:
//
//   FIT  1.8x, 1152x720. Fills the height and keeps the 16:10 aspect exactly
//        (640x400 and 1152x720 are both 16:10), leaving 64px bars at each end
//        of the long axis. 1.8 is not an integer, so a source pixel lands on
//        either one or two panel pixels and the dot pitch is visibly uneven.
//   DOT  1:1, centred. Every PC-98 pixel is exactly one panel pixel, so text
//        is as sharp as the panel allows and the blit is the cheapest it can
//        be — at the cost of using a quarter of the screen area.
//
// 2x would be the best of both, but 400*2 = 800 does not fit in 720; the only
// way to have it is to crop 80 lines off the PC-98 screen, which an emulator
// should not do.
enum {
    BRD_SCALE_FIT = 0,   // 1.8x, fills the height
    BRD_SCALE_DOT = 1,   // 1:1, centred
    BRD_SCALE_COUNT
};
#define BRD_SCALE_DEFAULT   BRD_SCALE_FIT

// FIT: 640*9/5 = 1152, 400*9/5 = 720. Kept as a rational so the blit steps
// with integer arithmetic and no floating point in the inner loop.
#define BRD_FIT_NUM         7
#define BRD_FIT_DEN         4

// ---- Audio ----------------------------------------------------------------
// The PC-98 sound path runs at 22050Hz stereo, same as the S3 forks. The codec
// itself (and the speaker enable on the I/O expander) is the BSP's business.
#define BRD_AUDIO_RATE      22050

// ---- Emulated CPU speed ----------------------------------------------------
// np2kai multiplies a 2.4576MHz base by np2cfg.multiple, and clamps nothing
// useful of its own (CPU_MULTIPLE_MAX is 2048). The ceiling is what this board
// can actually sustain in real time, which is far above the S3 forks' x5 - the
// menu steps through the ladder below rather than every integer, because the
// interesting range is wide and the row is a single cycling button. It stops at
// x10 because that is where the measured real-time headroom reaches zero -
// past it the machine simply runs slow, and the FM stream (which is clocked off
// emulated time) starves.
#define BRD_CPU_MULT_LADDER 1, 2, 3, 4, 5, 6, 8, 10
#define CPU_MULT_MAX        10
