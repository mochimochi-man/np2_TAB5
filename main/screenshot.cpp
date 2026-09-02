// Save the emulator's screen to the SD card as a PNG.
//
// The picture written is the PC-98 screen at its own size - 640x400 - not what
// the panel is showing. The panel image has been scaled by 1.75 and rotated a
// quarter turn to fit a 720x1280 display held sideways, and none of that is
// something anyone wants baked into a screenshot.
//
// What makes this possible without any cooperation from the emulator is that
// the menu runs in the emulator's own context, with the machine paused. The
// frame buffer np2kai last handed to lcd_blit() is therefore still exactly the
// screen that was showing when the menu was opened, and stays that way for as
// long as the menu is up - the menu draws into the PANEL buffer, not this one.
//
// The deflate compressor is miniz, which is in the ESP32-P4 boot ROM. Only the
// compressor, though: the ROM build defines MINIZ_NO_MALLOC, so every miniz
// entry point that allocates - including tdefl_write_image_to_png_file_in_
// memory_ex(), which would otherwise have written the whole file in one call -
// can never do anything but fail. What is left is tdefl_init() and
// tdefl_compress_buffer(), which take the caller's buffers and are what the
// IDF's own ROM test exercises, so the PNG container below is assembled here.
//
// A PNG is a signature, then chunks of [length, type, data, CRC-32]:
//
//   IHDR   width, height, bit depth 8, colour type 2 (RGB), no interlace
//   IDAT   a zlib stream of the rows, each preceded by its filter byte
//   IEND   empty
//
// The rows go in unfiltered (filter 0). A filter would compress better, but
// PC-98 screens are flat colour over large areas, which deflate already handles
// well, and an unfiltered row is one less thing to get wrong on a file format
// nothing here can open to check.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "esp_heap_caps.h"
#include "miniz.h"

extern "C" int ets_printf(const char *fmt, ...);

// lcd_mipi.cpp: the last frame the emulator handed over, and its geometry.
extern "C" const void *lcd_last_frame(int *w, int *h, int *stride);

#define SHOT_DIR "/sd"

// ---- PNG plumbing ----------------------------------------------------------
static uint32_t crc32_png(const uint8_t *d, size_t n, uint32_t crc) {
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int k = 0; k < 8; k++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xedb88320u) : (crc >> 1);
        }
    }
    return crc;
}

static void be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static bool write_chunk(FILE *f, const char *type, const uint8_t *data, size_t len) {
    uint8_t hdr[8];
    be32(hdr, (uint32_t)len);
    memcpy(hdr + 4, type, 4);
    if (fwrite(hdr, 1, 8, f) != 8) {
        return false;
    }
    if (len && fwrite(data, 1, len, f) != len) {
        return false;
    }
    uint32_t crc = crc32_png((const uint8_t *)type, 4, 0xffffffffu);
    crc = crc32_png(data, len, crc);
    uint8_t tail[4];
    be32(tail, ~crc);
    return fwrite(tail, 1, 4, f) == 4;
}

// ---- deflate sink ----------------------------------------------------------
// The IDAT chunk needs its length in front of it, so the compressed stream is
// collected whole before anything is written. Sized for the uncompressed input
// plus slack: deflate can grow incompressible data very slightly, and running
// out of room here would be a corrupt file rather than an error.
struct zsink {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool overflow;
};

static mz_bool z_put(const void *p, int len, void *user) {
    zsink *z = (zsink *)user;
    if (z->len + (size_t)len > z->cap) {
        z->overflow = true;
        return MZ_FALSE;
    }
    memcpy(z->buf + z->len, p, (size_t)len);
    z->len += (size_t)len;
    return MZ_TRUE;
}

// ---- file naming -----------------------------------------------------------
// Numbered rather than dated, like the floppy dumps: the number cannot collide
// and needs no clock, and the date is on the file itself because rtc_tab5_sync()
// has told the system what time it is.
static bool next_shot_name(char *out, size_t cap) {
    for (int i = 1; i <= 9999; i++) {
        snprintf(out, cap, "%s/SHOT%04d.PNG", SHOT_DIR, i);
        struct stat sb;
        if (stat(out, &sb) != 0) {
            return true;
        }
    }
    return false;
}

// ---- entry -----------------------------------------------------------------
// Writes the screenshot and returns its bare file name in `name` for the menu
// to show. Returns false if there was nothing to capture or nothing could be
// written.
extern "C" bool screenshot_save(char *name, size_t name_cap) {
    if (name && name_cap) {
        name[0] = 0;
    }
    int w = 0, h = 0, stride = 0;
    const uint16_t *src = (const uint16_t *)lcd_last_frame(&w, &h, &stride);
    if (!src || w <= 0 || h <= 0) {
        ets_printf("shot: nothing has been drawn yet\n");
        return false;
    }

    const size_t rowbytes = (size_t)w * 3;
    const size_t raw_len = (rowbytes + 1) * (size_t)h;   // a filter byte per row

    tdefl_compressor *comp =
        (tdefl_compressor *)heap_caps_calloc(1, sizeof(tdefl_compressor), MALLOC_CAP_SPIRAM);
    uint8_t *line = (uint8_t *)heap_caps_malloc(rowbytes, MALLOC_CAP_SPIRAM);
    zsink z = {};
    z.cap = raw_len + 4096;
    z.buf = (uint8_t *)heap_caps_malloc(z.cap, MALLOC_CAP_SPIRAM);
    bool ok = (comp && line && z.buf);

    if (ok) {
        ok = tdefl_init(comp, z_put, &z,
                        (int)TDEFL_WRITE_ZLIB_HEADER | (int)TDEFL_DEFAULT_MAX_PROBES)
             == TDEFL_STATUS_OKAY;
        if (!ok) {
            ets_printf("shot: tdefl_init failed\n");
        }
    }
    for (int y = 0; ok && y < h; y++) {
        const uint16_t *s = src + (size_t)y * stride;
        uint8_t *d = line;
        for (int x = 0; x < w; x++) {
            const uint16_t p = s[x];
            const uint8_t r = (uint8_t)((p >> 11) & 0x1f);
            const uint8_t g = (uint8_t)((p >> 5) & 0x3f);
            const uint8_t b = (uint8_t)(p & 0x1f);
            // The low bits are filled from the high ones rather than with
            // zeros, so that full scale stays full scale: 0x1f becomes 0xff,
            // not 0xf8, and white in the emulator is white in the file.
            *d++ = (uint8_t)((r << 3) | (r >> 2));
            *d++ = (uint8_t)((g << 2) | (g >> 4));
            *d++ = (uint8_t)((b << 3) | (b >> 2));
        }
        static const uint8_t filter_none = 0;
        ok = tdefl_compress_buffer(comp, &filter_none, 1, TDEFL_NO_FLUSH) == TDEFL_STATUS_OKAY &&
             tdefl_compress_buffer(comp, line, rowbytes, TDEFL_NO_FLUSH) == TDEFL_STATUS_OKAY;
    }
    if (ok) {
        ok = tdefl_compress_buffer(comp, nullptr, 0, TDEFL_FINISH) == TDEFL_STATUS_DONE;
        if (!ok) {
            ets_printf("shot: compression failed%s\n", z.overflow ? " (buffer too small)" : "");
        }
    }

    char path[64] = {0};
    if (ok) {
        ok = next_shot_name(path, sizeof(path));
    }
    if (ok) {
        FILE *f = fopen(path, "wb");
        ok = (f != nullptr);
        if (ok) {
            static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
            uint8_t ihdr[13];
            be32(ihdr + 0, (uint32_t)w);
            be32(ihdr + 4, (uint32_t)h);
            ihdr[8] = 8;      // bits per channel
            ihdr[9] = 2;      // colour type 2 = truecolour RGB
            ihdr[10] = 0;     // deflate
            ihdr[11] = 0;     // adaptive filtering
            ihdr[12] = 0;     // no interlace
            ok = fwrite(sig, 1, sizeof(sig), f) == sizeof(sig) &&
                 write_chunk(f, "IHDR", ihdr, sizeof(ihdr)) &&
                 write_chunk(f, "IDAT", z.buf, z.len) &&
                 write_chunk(f, "IEND", nullptr, 0);
            if (fclose(f) != 0) {
                ok = false;
            }
        }
        if (!ok) {
            ets_printf("shot: could not write %s\n", path);
        }
    }

    heap_caps_free(comp);
    heap_caps_free(line);
    heap_caps_free(z.buf);
    if (!ok) {
        return false;
    }

    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    ets_printf("shot: %s  %dx%d  %u bytes\n", base, w, h,
               (unsigned)(z.len + 8 + 25 + 12 + 12));
    if (name && name_cap) {
        snprintf(name, name_cap, "%s", base);
    }
    return true;
}
