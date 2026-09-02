// ESP32 platform layer for np2kai (i386c core) — everything except dosio.
// scrnmng is a REAL PSRAM framebuffer (640x400 RGB565, later → LCD).
// Sound / MIDI / serial / input / font are minimal stubs for the first boot.
// extern "C" symbols are resolved by name at link, so stub signatures only need
// to be self-consistent + match the core's call ABI (pointer handles == void*).

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"

extern "C" int ets_printf(const char *fmt, ...);
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <time.h>
#include <cstddef>
#include <cstdlib>
#include <new>

// The core's C++ parts (fmgen OPNA) do large new[] (e.g. a 256KB ADPCM buffer).
// Internal DRAM is tiny; route large allocations to PSRAM so they don't throw
// bad_alloc. heap_caps_free handles both PSRAM and internal pointers.
static inline void *np2_cpp_alloc(std::size_t n) {
    void *p = nullptr;
    if (n >= 4096) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = malloc(n);
    if (!p) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p;
}
static void np2_alloc_fail(std::size_t n) {
    ets_printf("[NEW FAIL] size=%u  free_psram=%u free_internal=%u free_8bit=%u\n",
               (unsigned)n, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
}
// NOTE: returns NULL on failure instead of throwing/aborting. np2's C++ code
// (fmgen OPNA) checks `if(!buf) return false`, so FM auto-disables when PSRAM is
// exhausted — i.e. "enable FM if it fits", degrade gracefully otherwise.
void *operator new(std::size_t n)   { void *p = np2_cpp_alloc(n); if (!p) np2_alloc_fail(n); return p; }
void *operator new[](std::size_t n) { void *p = np2_cpp_alloc(n); if (!p) np2_alloc_fail(n); return p; }
void *operator new(std::size_t n, const std::nothrow_t &) noexcept   { return np2_cpp_alloc(n); }
void *operator new[](std::size_t n, const std::nothrow_t &) noexcept { return np2_cpp_alloc(n); }
void operator delete(void *p) noexcept   { heap_caps_free(p); }
void operator delete[](void *p) noexcept { heap_caps_free(p); }
void operator delete(void *p, std::size_t) noexcept   { heap_caps_free(p); }
void operator delete[](void *p, std::size_t) noexcept { heap_caps_free(p); }

extern "C" {
#include <compiler.h>
#include "scrnmng.h"   // SCRNMNG, SCRNSURF, RGB16, RGB32
#include "timemng.h"   // _SYSTIME
#include "commng.h"    // COMMNG (serial/MIDI manager interface)
}

extern "C" {

// ================= scrnmng — real framebuffer (640x480 RGB565 in PSRAM) =======
SCRNMNG scrnmng;
static SCRNSURF s_surf;
static uint8_t *s_fb = nullptr;
#define PC98_W 640
#define PC98_H 480   /* >= any PC-98 mode we render (400/480), so sdraw never overflows */

// Allocate the framebuffer up-front while PSRAM is still plentiful (call before
// pccore_init). Lazy alloc during drawscreen could fail once the heap is tight
// → NULL surface ptr → crash in sdraw. Returns true on success.
bool pc98_scrnmng_init(void) {
    if (!s_fb)
        // 128-byte aligned: this is the PPA's input picture, and the PPA
        // driver requires external buffers aligned to the L2 cache line both in
        // address and in size (640*480*2 is already a multiple of 128).
        s_fb = (uint8_t *)heap_caps_aligned_alloc(128, (size_t)PC98_W * PC98_H * 2,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fb) memset(s_fb, 0, (size_t)PC98_W * PC98_H * 2);
    return s_fb != nullptr;
}

const SCRNSURF *scrnmng_surflock(void) {
    if (!s_fb) pc98_scrnmng_init();
    s_surf.ptr    = s_fb;
    s_surf.xalign = 2;              // bytes/pixel (16bpp)
    s_surf.yalign = PC98_W * 2;     // bytes/line
    s_surf.width  = PC98_W;
    s_surf.height = PC98_H;
    s_surf.bpp    = 16;
    s_surf.extend = 0;
    return &s_surf;
}
void scrnmng_surfunlock(const SCRNSURF *surf) { (void)surf; }
void scrnmng_setwidth(int posx, int width)  { (void)posx; (void)width; }
void scrnmng_setheight(int posy, int height) { (void)posy; (void)height; }
RGB16 scrnmng_makepal16(RGB32 c) {
    return (RGB16)(((c.p.r >> 3) << 11) | ((c.p.g >> 2) << 5) | (c.p.b >> 3));
}
uint8_t *pc98_framebuffer(void) { return s_fb; }   // for the LCD blitter later

// ================= time =================
BRESULT timemng_gettime(_SYSTIME *t) {
    if (t) {
        time_t now = time(nullptr);
        struct tm *lt = localtime(&now);
        if (lt) {
            t->year = (UINT16)(lt->tm_year + 1900); t->month = (UINT16)(lt->tm_mon + 1);
            t->week = (UINT16)lt->tm_wday;          t->day = (UINT16)lt->tm_mday;
            t->hour = (UINT16)lt->tm_hour;          t->minute = (UINT16)lt->tm_min;
            t->second = (UINT16)lt->tm_sec;         t->milli = 0;
        }
    }
    return SUCCESS;
}
int64_t cpu_features_get_time_usec(void) { return (int64_t)esp_timer_get_time(); }

// gettimeofday: ESP-IDF newlib usually provides this; define defensively.
int _np2_gettimeofday(struct timeval *tv, void *tz) {
    (void)tz;
    if (tv) { int64_t us = esp_timer_get_time(); tv->tv_sec = us / 1000000; tv->tv_usec = us % 1000000; }
    return 0;
}

// ================= sysmng / taskmng (no-op) =================
void sysmng_cpureset(void) {}
void sysmng_update(UINT update) { (void)update; }
void sysmng_updatecaption(UINT8 flag) { (void)flag; }
void taskmng_exit(void) {}

// ================= sound =================
// When FM sound is enabled (main.cpp sets g_sound_enable before pccore_init),
// soundmng_create returns the per-buffer sample count so sound_create() allocates
// the PCM stream; otherwise 0 keeps sound disabled (sound_create returns FAILURE).
int g_sound_enable = 0;
UINT    soundmng_create(UINT rate, UINT ms) {
    if (!g_sound_enable) return 0;
    if (ms == 0) ms = 20;
    return (rate * ms) / 1000;      // stereo frames per pcmlock block
}
void    soundmng_destroy(void) {}
void    soundmng_reset(void) {}
void    soundmng_play(void) {}
void    soundmng_stop(void) {}
void    soundmng_sync(void) {}
void    soundmng_setreverse(BOOL reverse) { (void)reverse; }
BRESULT soundmng_pcmplay(UINT num, BOOL loop) { (void)num; (void)loop; return FAILURE; }
void    soundmng_pcmstop(UINT num) { (void)num; }

// getsnd (PCM decode) — unused without sound
void *getsnd_create(void *datptr, UINT datsize) { (void)datptr; (void)datsize; return nullptr; }
void  getsnd_destroy(void *hdl) { (void)hdl; }
UINT  getsnd_getpcmbyleng(void *hdl, void *pcm, UINT leng) { (void)hdl; (void)pcm; (void)leng; return 0; }
BRESULT getsnd_setmixproc(void *snd, UINT samprate, UINT channels) { (void)snd; (void)samprate; (void)channels; return FAILURE; }

// ================= MIDI (vermouth) — silent =================
void *midimod_create(UINT samprate) { (void)samprate; return nullptr; }
void  midimod_destroy(void *hdl) { (void)hdl; }
void  midimod_loadall(void *hdl) { (void)hdl; }
void *midiout_create(void *mod, UINT worksize) { (void)mod; (void)worksize; return nullptr; }
void  midiout_destroy(void *hdl) { (void)hdl; }
const void *midiout_get(void *hdl, UINT *samples) { (void)hdl; if (samples) *samples = 0; return nullptr; }
void  midiout_longmsg(void *hdl, const void *msg, UINT size) { (void)hdl; (void)msg; (void)size; }
void  midiout_shortmsg(void *hdl, UINT32 msg) { (void)hdl; (void)msg; }

// ================= COM (serial / MIDI) — always "not connected" ===============
// No real UART/MIDI backend on this board. Returning NULL crashed the core: e.g.
// mpu98ii_i0() does `cm = commng_create(...); if (cm->connect != OFF)` with no NULL
// check, so a NULL blew up (LoadProhibited) the moment PM2 probed the MPU-98II MIDI
// port. Instead hand back a shared, stateless dummy whose connect == COMCONNECT_OFF
// and whose methods are safe no-ops — every device then reads as "absent".
static UINT   com_read(COMMNG s, UINT8 *d)      { (void)s; if (d) *d = 0xff; return 0; }
static UINT   com_write(COMMNG s, UINT8 d)      { (void)s; (void)d; return 1; }
static UINT   com_writeretry(COMMNG s)          { (void)s; return 1; }
static void   com_beginblock(COMMNG s)          { (void)s; }
static void   com_endblock(COMMNG s)            { (void)s; }
static UINT   com_lastwrite(COMMNG s)           { (void)s; return 1; }
static UINT8  com_getstat(COMMNG s)             { (void)s; return 0; }
static INTPTR com_msg(COMMNG s, UINT m, INTPTR p){ (void)s; (void)m; (void)p; return 0; }
static void   com_release(COMMNG s)             { (void)s; }
static _COMMNG s_com_off = {
    COMCONNECT_OFF, com_read, com_write, com_writeretry, com_beginblock,
    com_endblock, com_lastwrite, com_getstat, com_msg, com_release, 0, 0, 0
};
COMMNG commng_create(UINT device, BOOL onReset) { (void)device; (void)onReset; return &s_com_off; }
void   commng_destroy(COMMNG hdl) { (void)hdl; }   // shared static: nothing to free

// ================= font manager (OSD/menu) — unused for PC-98 text =================
void *fontmng_create(int size, UINT type, const char *fontface) { (void)size; (void)type; (void)fontface; return nullptr; }
void  fontmng_destroy(void *hdl) { (void)hdl; }
void *fontmng_get(void *hdl, const char *str) { (void)hdl; (void)str; return nullptr; }

// ================= input =================
// Real USB mouse only (HID boot protocol; state comes from usb_kbd.cpp's
// g_umouse_*). Buttons are ACTIVE-LOW on the uPD8255 port (bit SET=released,
// CLEAR=pressed); idle = LEFT(0x80)|RIGHT(0x20) = 0xA0. (The old stub returned
// 0 = both buttons stuck down -> auto-click.)
extern volatile int16_t g_umouse_dx, g_umouse_dy;  // usb_kbd.cpp: real USB mouse
extern volatile uint8_t g_umouse_btn;
extern volatile int     g_umouse_present;
BYTE  mousemng_getstat(SINT16 *x, SINT16 *y, int clear) {
    if (!g_umouse_present) {           // no mouse attached: neutral state
        if (x) *x = 0;
        if (y) *y = 0;
        (void)clear;
        return 0xA0;                   // both buttons released (active low)
    }
    if (x) *x = (SINT16)g_umouse_dx;
    if (y) *y = (SINT16)g_umouse_dy;
    if (clear) { g_umouse_dx = 0; g_umouse_dy = 0; }
    return g_umouse_btn;
}
// Joystick bits are ACTIVE-LOW (core does ret &= joymng_getstat(); 0x00 would mean
// every direction+button held -> games ignore the keyboard). 0xFF = nothing pressed.
UINT8 joymng_getstat(void) { return 0xff; }

// ================= misc =================
void wabrly_initialize(void) {}

} // extern "C"
