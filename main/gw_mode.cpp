// Greaseweazle over the USB-A port: read a real floppy in the drive and write
// it to the SD card as a disk image.
//
// The Greaseweazle is a USB CDC-ACM device that drives the floppy and hands
// back RAW FLUX - the time between magnetic transitions, nothing more. Every
// part of turning that into sectors (clock recovery, MFM decode, address mark
// search, CRC) is the job of the host; in the stock toolchain that host is
// Python on a PC, and here it has to be this file.
//
// The protocol, from the reference implementation (greaseweazle/usb.py):
//
//   command   [cmd, len, ...args]        len counts the whole packet
//   reply     [cmd, status]              status 0 = Okay
//
//   GetInfo        [0, 3, 0]  then 32 bytes  <4BI4B3H14x
//   Seek           [2, 3, cyl]
//   Head           [3, 3, head]
//   SetParams      [4, 13, 0, select_us, step_us, settle_ms, motor_ms, wdog_ms]
//   Motor          [6, 4, unit, on]
//   ReadFlux       [7, 8, ticks:u32, revs:u16]   then a flux stream, then 0
//   GetFluxStatus  [9, 2]
//   Select         [12, 3, unit]
//   Deselect       [13, 2]
//   SetBusType     [14, 3, type]        1 = IBM PC, which "--drive a" selects
//
// The flux stream itself:
//
//   1..249    that many ticks since the previous transition
//   250..254  first byte of a two-byte value: 250 + (b-250)*255 + next - 1
//   255       escape: next byte is an opcode (1 = Index, 2 = Space), followed
//             by a 28-bit value spread over four bytes, seven bits each in
//             bits 1..7 - so none of those four bytes can be zero
//   0         end of stream, and the only place a zero can appear
//
// The whole path is here: identify the device, spin the drive, capture flux,
// recover the bitstream, find the sectors, and write the disk out as a raw
// .HDM that the emulator can mount straight from the menu.

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "usb/cdc_acm_host.h"

extern "C" int ets_printf(const char *fmt, ...);

// The vendor/product pair registered with pid.codes for the Greaseweazle.
#define GW_VID 0x1209
#define GW_PID 0x4D69

enum {
    CMD_GETINFO       = 0,
    CMD_SEEK          = 2,
    CMD_HEAD          = 3,
    CMD_SETPARAMS     = 4,
    CMD_MOTOR         = 6,
    CMD_READFLUX      = 7,
    CMD_GETFLUXSTATUS = 9,
    CMD_SELECT        = 12,
    CMD_DESELECT      = 13,
    CMD_SETBUSTYPE    = 14,
};

#define BUS_IBMPC 1

// Three revolutions of flux at 500kbps is a little over 200KB; a megabyte
// leaves room for a slow disk and for the escape sequences. PSRAM - this is
// streamed into by a task, never an ISR.
#define GW_RX_CAP (1024 * 1024)

static cdc_acm_dev_hdl_t s_dev = nullptr;
static uint8_t *s_rx = nullptr;
static volatile size_t s_rx_len = 0;      // bytes written by the USB callback
static size_t s_rx_pos = 0;               // bytes consumed by the reader
static volatile bool s_rx_ovf = false;
static SemaphoreHandle_t s_rx_sig = nullptr;
static uint32_t s_freq = 0;               // sample ticks per second

// ---- transport -------------------------------------------------------------
// Append-only: the callback is the sole writer and only ever advances s_rx_len,
// the reader is the sole reader and only ever advances s_rx_pos behind it. That
// removes the need for a lock on the data itself, which matters because these
// buffers arrive a thousand times a second during a track read.
static bool rx_cb(const uint8_t *data, size_t len, void *arg) {
    (void)arg;
    const size_t room = GW_RX_CAP - s_rx_len;
    if (len > room) {
        s_rx_ovf = true;
        len = room;
    }
    memcpy(s_rx + s_rx_len, data, len);
    s_rx_len = s_rx_len + len;
    xSemaphoreGive(s_rx_sig);
    return true;   // we took the buffer
}

static void rx_reset(void) {
    s_rx_len = 0;
    s_rx_pos = 0;
    s_rx_ovf = false;
    xSemaphoreTake(s_rx_sig, 0);
}

// Wait for `want` more bytes and take them. The device answers a command within
// a millisecond or two; a read that times out means the command was not
// understood, which is worth saying rather than hanging.
static bool rx_take(uint8_t *out, size_t want, int timeout_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        if (s_rx_len - s_rx_pos >= want) {
            memcpy(out, s_rx + s_rx_pos, want);
            s_rx_pos += want;
            return true;
        }
        if (xTaskGetTickCount() >= deadline) {
            return false;
        }
        xSemaphoreTake(s_rx_sig, pdMS_TO_TICKS(20));
    }
}

// Scan forward for the stream terminator, waiting for more data as needed.
static bool rx_wait_zero(size_t *end, int timeout_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    size_t scan = s_rx_pos;
    for (;;) {
        const size_t have = s_rx_len;
        while (scan < have) {
            if (s_rx[scan] == 0) {
                *end = scan;
                return true;
            }
            scan++;
        }
        if (s_rx_ovf) {
            ets_printf("gw: receive buffer overflowed\n");
            return false;
        }
        if (xTaskGetTickCount() >= deadline) {
            return false;
        }
        xSemaphoreTake(s_rx_sig, pdMS_TO_TICKS(20));
    }
}

// Send a command and check the acknowledgement. Every Greaseweazle command
// answers with its own opcode and a status byte, so a mismatched opcode means
// the stream has lost sync rather than that the command failed.
static bool gw_cmd(const uint8_t *cmd, size_t len, int timeout_ms = 1000) {
    rx_reset();
    if (cdc_acm_host_data_tx_blocking(s_dev, cmd, len, 1000) != ESP_OK) {
        ets_printf("gw: tx failed (cmd %u)\n", cmd[0]);
        return false;
    }
    uint8_t ack[2];
    if (!rx_take(ack, sizeof(ack), timeout_ms)) {
        ets_printf("gw: no reply to cmd %u\n", cmd[0]);
        return false;
    }
    if (ack[0] != cmd[0]) {
        ets_printf("gw: out of sync (got %02x, sent %02x)\n", ack[0], cmd[0]);
        return false;
    }
    if (ack[1] != 0) {
        ets_printf("gw: cmd %u failed, status %u\n", cmd[0], ack[1]);
        return false;
    }
    return true;
}

static bool gw_cmd2(uint8_t c) {
    const uint8_t b[2] = {c, 2};
    return gw_cmd(b, 2);
}
static bool gw_cmd3(uint8_t c, uint8_t a) {
    const uint8_t b[3] = {c, 3, a};
    return gw_cmd(b, 3);
}
// Turning the motor on does not answer until the drive is up to speed - the
// spin-up delay set by gw_set_delays() is spent inside this command, not after
// it - so this one needs a timeout longer than the delay itself.
static bool gw_motor(uint8_t unit, bool on) {
    const uint8_t b[4] = {CMD_MOTOR, 4, unit, (uint8_t)(on ? 1 : 0)};
    return gw_cmd(b, 4, 5000);
}

// ---- device info -----------------------------------------------------------
static const char *hw_model_name(uint8_t model) {
    switch (model) {
    case 1:  return "F1";
    case 4:  return "F7";
    case 7:  return "AT32F4";
    default: return "?";
    }
}

static bool gw_get_info(void) {
    const uint8_t cmd[3] = {CMD_GETINFO, 3, 0 /*Firmware*/};
    if (!gw_cmd(cmd, sizeof(cmd))) {
        return false;
    }
    uint8_t d[32];
    if (!rx_take(d, sizeof(d), 1000)) {
        ets_printf("gw: GetInfo returned no data\n");
        return false;
    }
    // <4B I 4B 3H 14x
    const uint8_t major = d[0], minor = d[1], is_main = d[2], max_cmd = d[3];
    s_freq = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
             ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    const uint8_t hw_model = d[8], hw_submodel = d[9], usb_speed = d[10];
    const uint16_t mcu_mhz = (uint16_t)(d[12] | (d[13] << 8));
    const uint16_t mcu_sram_kb = (uint16_t)(d[14] | (d[15] << 8));
    const uint16_t usb_buf_kb = (uint16_t)(d[16] | (d[17] << 8));

    ets_printf("gw: firmware %u.%u (%s), max_cmd=%u\n",
               major, minor, is_main ? "main" : "bootloader", max_cmd);
    ets_printf("gw: hw %s (model %u.%u), USB %s\n",
               hw_model_name(hw_model), hw_model, hw_submodel,
               usb_speed ? "high speed" : "full speed");
    // No floats in ets_printf, so the tick period is printed in picoseconds.
    ets_printf("gw: sample_freq %u Hz (%u ps per tick)\n",
               (unsigned)s_freq,
               s_freq ? (unsigned)(1000000000000ULL / s_freq) : 0u);
    ets_printf("gw: mcu %u MHz, %u KB SRAM, %u KB USB buffer\n",
               mcu_mhz, mcu_sram_kb, usb_buf_kb);
    return s_freq != 0;
}

// The drive timings, which the device keeps in its own settings until told
// otherwise. The one that matters is the motor delay: a 5.25-inch spindle does
// not reach speed in the stock 750ms, and reading before it does gives nothing
// but "No Index". Setting it here rather than relying on whatever the device
// was last told means any Greaseweazle works, not just one that has been
// prepared on a PC first.
static bool gw_set_delays(void) {
    static const uint16_t d[5] = {
        10,      // select, us
        10000,   // step, us
        15,      // seek settle, ms
        2000,    // motor spin-up, ms - the whole point of this
        10000,   // watchdog, ms
    };
    uint8_t cmd[13] = {CMD_SETPARAMS, 13, 0 /*Params.Delays*/};
    for (int i = 0; i < 5; i++) {
        cmd[3 + i * 2] = (uint8_t)(d[i] & 0xff);
        cmd[4 + i * 2] = (uint8_t)(d[i] >> 8);
    }
    return gw_cmd(cmd, sizeof(cmd));
}

// ---- flux ------------------------------------------------------------------
#define HIST_BUCKETS 64          // a quarter of a microsecond each, so 0..16us

struct flux_stats {
    uint32_t transitions;
    uint32_t index_ticks[8];     // ticks since the previous index pulse
    int      nindex;
    uint32_t hist[HIST_BUCKETS];
};

typedef void (*flux_sink_t)(uint32_t ticks, void *arg);

static bool decode_flux(const uint8_t *p, size_t n, flux_stats &st,
                        flux_sink_t sink = nullptr, void *arg = nullptr) {
    const uint32_t bucket = s_freq / 4000000;    // ticks per 0.25us
    memset(&st, 0, sizeof st);
    if (bucket == 0) {
        return false;
    }
    size_t i = 0;
    uint32_t ticks = 0;              // accumulated since the last transition
    int64_t since_index = 0;         // may go negative, exactly as upstream
    while (i < n) {
        const uint8_t b = p[i++];
        if (b == 255) {
            if (i + 5 > n) {
                return false;
            }
            const uint8_t op = p[i++];
            const uint32_t v = ((uint32_t)(p[i + 0] & 254) >> 1) |
                               ((uint32_t)(p[i + 1] & 254) << 6) |
                               ((uint32_t)(p[i + 2] & 254) << 13) |
                               ((uint32_t)(p[i + 3] & 254) << 20);
            i += 4;
            if (op == 1) {                       // Index
                if (st.nindex < 8) {
                    st.index_ticks[st.nindex] = (uint32_t)(since_index + ticks + v);
                }
                st.nindex++;
                since_index = -(int64_t)(ticks + v);
            } else if (op == 2) {                // Space
                ticks += v;
            } else {
                ets_printf("gw: bad opcode %u in flux stream\n", op);
                return false;
            }
        } else {
            uint32_t v;
            if (b < 250) {
                v = b;
            } else {
                if (i >= n) {
                    return false;
                }
                v = 250 + (uint32_t)(b - 250) * 255 + p[i++] - 1;
            }
            ticks += v;
            st.transitions++;
            if (sink) {
                sink(ticks, arg);
            }
            const uint32_t k = ticks / bucket;
            st.hist[(k < HIST_BUCKETS) ? k : (HIST_BUCKETS - 1)]++;
            since_index += ticks;
            ticks = 0;
        }
    }
    return true;
}

// ---- MFM -------------------------------------------------------------------
// Flux intervals become a bitstream, the bitstream becomes sectors.
//
// At 500kbps MFM one raw cell is 1us and transitions come 2, 3 or 4 cells
// apart - which is exactly the 2.0/3.0/4.0us the histogram above shows. The
// cell time is NOT assumed: it is measured from the shortest peak, so a 2DD
// disk (twice the cell time) or a drive running at the wrong speed decodes
// just as well, and the numbers that come out say which it was.
//
// Sectors are found by their address marks. Three 0xA1 bytes written with a
// missing clock bit encode as raw 0x4489 each - a pattern that cannot occur in
// legally encoded data - and the byte after them says what follows:
//
//   0xFE  ID    cyl, head, sec, size, crc16
//   0xFB  data  128<<size bytes, crc16      (0xF8 = deleted data)
//
// Both CRCs cover the three 0xA1 bytes and the mark as well, and a correct one
// leaves a running CRC-CCITT of zero.

#define MAX_BITS (2048 * 1024)          // enough for five revolutions at 500kbps
#define MAX_SECT 32
#define SECT_STRIDE 1024        // the largest sector this decodes

static uint8_t *s_bits = nullptr;

// Clock recovery. The flux stream says when transitions happened; this decides
// which bit cells they landed in, which is the whole difficulty of reading a
// floppy - the disk is turning at whatever speed it feels like and the cell
// boundaries have to be inferred from the data itself.
//
// Rounding each interval to the nearest whole number of cells is not enough.
// It throws away the leftover, so the decoder never learns that it is running
// slightly early or late, and on the inner cylinders - where the bits are
// packed tightest and the margins are thinnest - it loses sectors that are
// perfectly readable. Carrying the phase error forward and correcting a
// fraction of it each time is what the reference implementation does
// (greaseweazle/track.py), and these are its constants.
#define PLL_PERIOD_ADJ 0.05f      // how fast the clock period tracks the disk
#define PLL_PHASE_ADJ  0.60f      // how much of the phase error to take out

struct pll_state {
    float    cell;                 // current clock period, in ticks
    float    centre;               // what the histogram said it should be
    float    phase;                // error carried forward, in ticks
    uint32_t nbits;
};

static void pll_sink(uint32_t flux, void *arg) {
    pll_state *st = (pll_state *)arg;
    st->phase += (float)flux;
    if (st->phase < st->cell / 2) {
        return;                              // too short to be a transition
    }
    // Clock out zero or more 0s, then the 1 this transition represents.
    int zeros = 0;
    for (;;) {
        st->phase -= st->cell;
        if (st->phase < st->cell / 2) {
            break;
        }
        zeros++;
        if (st->nbits < MAX_BITS) {
            s_bits[st->nbits++] = 0;
        }
    }
    if (st->nbits < MAX_BITS) {
        s_bits[st->nbits++] = 1;
    }
    // In sync, follow the disk; out of sync, fall back towards the nominal
    // rate rather than chasing whatever the noise happened to say.
    if (zeros <= 3) {
        st->cell += st->phase * PLL_PERIOD_ADJ;
    } else {
        st->cell += (st->centre - st->cell) * PLL_PERIOD_ADJ;
    }
    if (st->cell < st->centre * 0.9f) {
        st->cell = st->centre * 0.9f;
    }
    if (st->cell > st->centre * 1.1f) {
        st->cell = st->centre * 1.1f;
    }
    st->phase *= (1.0f - PLL_PHASE_ADJ);
}

// The centre of the shortest strong peak is two cells wide, so half of it is
// the cell time. Weighting across the peak recovers more precision than the
// quarter-microsecond buckets have on their own.
static float estimate_cell(const flux_stats &st) {
    const uint32_t bucket = s_freq / 4000000;
    const uint32_t floor_count = st.transitions / 20;
    int k0 = -1, k1 = -1;
    for (int k = 1; k < HIST_BUCKETS; k++) {
        if (st.hist[k] > floor_count) {
            if (k0 < 0) {
                k0 = k;
            }
            k1 = k;
        } else if (k0 >= 0) {
            break;                       // past the first peak
        }
    }
    if (k0 < 0) {
        return 0;
    }
    uint64_t num = 0, den = 0;
    for (int k = k0; k <= k1; k++) {
        num += (uint64_t)st.hist[k] * (uint64_t)(2 * k + 1) * bucket;
        den += st.hist[k];
    }
    return den ? (float)num / (float)(den * 4) : 0.0f;
}

static uint16_t crc16_ccitt(const uint8_t *d, size_t n, uint16_t crc) {
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int k = 0; k < 8; k++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Raw cells alternate clock, data, clock, data; only the data bits carry the
// byte. Advances *p past the sixteen cells it consumed.
static uint8_t mfm_byte(uint32_t *p, uint32_t nbits) {
    uint8_t b = 0;
    for (int k = 0; k < 8; k++) {
        if (*p + 2 > nbits) {
            return (uint8_t)(b << (8 - k));
        }
        (*p)++;                          // the clock bit carries nothing
        b = (uint8_t)((b << 1) | s_bits[(*p)++]);
    }
    return b;
}

struct sect_hit {
    uint8_t  c, h, r, n;
    bool     id_ok;
    bool     data_ok;
    bool     deleted;
    int      seen;
};

static int sect_find(const sect_hit *v, int cnt, uint8_t c, uint8_t h, uint8_t r) {
    for (int k = 0; k < cnt; k++) {
        if (v[k].c == c && v[k].h == h && v[k].r == r) {
            return k;
        }
    }
    return -1;
}

// Walks the bitstream once, collecting every sector it can find. Sectors
// repeat once per revolution, so a hit that failed its CRC on one pass can be
// replaced by a good copy from the next.
static int mfm_scan(uint32_t nbits, sect_hit *out, int max_out,
                    uint8_t *data, size_t dcap) {
    static const uint8_t a1[3] = {0xa1, 0xa1, 0xa1};
    int cnt = 0;
    int pending = -1;                    // slot whose data field is next
    uint64_t sr = 0;

    for (uint32_t i = 0; i < nbits; i++) {
        sr = (sr << 1) | s_bits[i];
        if ((sr & 0xffffffffffffULL) != 0x448944894489ULL) {
            continue;
        }
        uint32_t p = i + 1;
        const uint8_t mark = mfm_byte(&p, nbits);

        if (mark == 0xfe) {                          // ID address mark
            uint8_t id[6];
            for (int k = 0; k < 6; k++) {
                id[k] = mfm_byte(&p, nbits);
            }
            uint16_t crc = crc16_ccitt(a1, 3, 0xffff);
            crc = crc16_ccitt(&mark, 1, crc);
            crc = crc16_ccitt(id, 6, crc);
            if (crc != 0) {
                pending = -1;                        // do not trust the header
                continue;
            }
            int k = sect_find(out, cnt, id[0], id[1], id[2]);
            if (k < 0) {
                if (cnt >= max_out) {
                    continue;
                }
                k = cnt++;
                memset(&out[k], 0, sizeof(out[k]));
                out[k].c = id[0];
                out[k].h = id[1];
                out[k].r = id[2];
                out[k].n = id[3];
            }
            out[k].id_ok = true;
            out[k].seen++;
            pending = k;

        } else if (mark == 0xfb || mark == 0xf8) {   // data address mark
            if (pending < 0) {
                continue;
            }
            const int slot = pending;
            pending = -1;
            const uint32_t len = 128u << (out[slot].n & 7);
            if (out[slot].data_ok) {
                continue;                            // already have a good copy
            }
            // Decode into the slot even before the CRC is known: if it fails,
            // the next revolution overwrites it, and if none pass, a bad copy
            // still beats a hole in the image.
            uint8_t *dst = (data && len <= SECT_STRIDE &&
                            (size_t)(slot + 1) * SECT_STRIDE <= dcap)
                           ? data + (size_t)slot * SECT_STRIDE : nullptr;
            uint16_t crc = crc16_ccitt(a1, 3, 0xffff);
            crc = crc16_ccitt(&mark, 1, crc);
            for (uint32_t k = 0; k < len; k++) {
                const uint8_t b = mfm_byte(&p, nbits);
                crc = crc16_ccitt(&b, 1, crc);
                if (dst) {
                    dst[k] = b;
                }
            }
            uint8_t tail[2];
            tail[0] = mfm_byte(&p, nbits);
            tail[1] = mfm_byte(&p, nbits);
            crc = crc16_ccitt(tail, 2, crc);
            out[slot].data_ok = (crc == 0);
            out[slot].deleted = (mark == 0xf8);
            i = p;                                   // skip past what we read
            sr = 0;
        }
    }
    return cnt;
}

// ---- a track at a time -----------------------------------------------------
struct track_out {
    sect_hit hit[MAX_SECT];
    int      nsect;
    uint8_t *data;              // MAX_SECT * SECT_STRIDE, indexed by slot
    float    cell;
    unsigned kbps;
    unsigned rpm10;
};

// Recover the bitstream from a flux capture and pick the sectors out of it.
static bool track_decode(const uint8_t *stream, size_t n, const flux_stats &st,
                         track_out &out, bool verbose, float cell_bias) {
    out.nsect = 0;
    out.cell = estimate_cell(st) * cell_bias;
    if (out.cell < 4.0f) {
        if (verbose) {
            ets_printf("gw:   no usable cell time - blank or unformatted track\n");
        }
        return false;
    }
    out.kbps = (unsigned)((float)s_freq / (2.0f * out.cell) / 1000.0f);
    if (verbose) {
        const unsigned cell_ns = (unsigned)(out.cell * 1000000000.0f / (float)s_freq);
        ets_printf("gw:   cell %u.%u ticks = %u ns  ->  %u kbps  (%s)\n",
                   (unsigned)out.cell, ((unsigned)(out.cell * 10)) % 10,
                   cell_ns, out.kbps, (out.kbps > 375) ? "2HD" : "2DD");
    }

    if (!s_bits) {
        s_bits = (uint8_t *)heap_caps_malloc(MAX_BITS, MALLOC_CAP_SPIRAM);
        if (!s_bits) {
            ets_printf("gw:   out of memory for the bitstream\n");
            return false;
        }
    }
    pll_state pll = {out.cell, out.cell, 0.0f, 0};
    flux_stats ignored;
    if (!decode_flux(stream, n, ignored, pll_sink, &pll)) {
        return false;
    }
    if (out.data) {
        memset(out.data, 0, (size_t)MAX_SECT * SECT_STRIDE);
    }
    out.nsect = mfm_scan(pll.nbits, out.hit, MAX_SECT,
                         out.data, (size_t)MAX_SECT * SECT_STRIDE);

    if (verbose) {
        int good = 0;
        ets_printf("gw:   %u bits, %d sectors:\n", (unsigned)pll.nbits, out.nsect);
        for (int k = 0; k < out.nsect; k++) {
            ets_printf("gw:     C%u H%u R%u N%u (%u bytes) x%d  id=%s data=%s%s\n",
                       out.hit[k].c, out.hit[k].h, out.hit[k].r, out.hit[k].n,
                       128u << (out.hit[k].n & 7), out.hit[k].seen,
                       out.hit[k].id_ok ? "ok" : "BAD",
                       out.hit[k].data_ok ? "ok" : "BAD",
                       out.hit[k].deleted ? " (deleted)" : "");
            if (out.hit[k].data_ok) {
                good++;
            }
        }
        ets_printf("gw:   %d of %d sectors read cleanly\n", good, out.nsect);
    }
    return out.nsect > 0;
}

// Seek to a track, capture it and decode it. The drive must already be
// selected with its motor running.
//
// A revolution count of N stops at the Nth index pulse, and the capture starts
// wherever the head happens to be - so N revolutions yields only N-1 complete
// ones. Asking for three is what gives every sector two chances to be read.
static bool track_read(int cyl, int head, int revs, track_out &out, bool verbose,
                       float cell_bias = 1.0f) {
    out.nsect = 0;
    if (!gw_cmd3(CMD_SEEK, (uint8_t)cyl) || !gw_cmd3(CMD_HEAD, (uint8_t)head)) {
        return false;
    }
    const uint8_t rf[8] = {CMD_READFLUX, 8, 0, 0, 0, 0,
                           (uint8_t)(revs & 0xff), (uint8_t)(revs >> 8)};
    if (!gw_cmd(rf, sizeof(rf))) {
        return false;
    }
    size_t end = 0;
    bool ok = rx_wait_zero(&end, 5000);
    if (!ok) {
        ets_printf("gw: flux stream did not terminate\n");
    } else {
        const uint8_t *p = s_rx + s_rx_pos;
        const size_t n = end - s_rx_pos;
        s_rx_pos = end + 1;

        flux_stats st;
        ok = decode_flux(p, n, st);
        if (!ok) {
            ets_printf("gw: could not decode the flux stream\n");
        } else {
            out.rpm10 = (st.nindex > 1 && st.index_ticks[1])
                        ? (unsigned)((uint64_t)600 * s_freq / st.index_ticks[1]) : 0;
            if (verbose) {
                ets_printf("gw: cyl %d head %d: %u flux bytes, %u transitions, %d index\n",
                           cyl, head, (unsigned)n, (unsigned)st.transitions, st.nindex);
                for (int k = 1; k < st.nindex && k < 8; k++) {
                    const uint32_t t = st.index_ticks[k];
                    if (!t) {
                        continue;
                    }
                    const unsigned ms100 = (unsigned)((uint64_t)t * 100000ULL / s_freq);
                    ets_printf("gw:   rev %d: %u.%02u ms  =  %u.%u rpm\n", k,
                               ms100 / 100, ms100 % 100,
                               (unsigned)((uint64_t)600 * s_freq / t) / 10,
                               (unsigned)((uint64_t)600 * s_freq / t) % 10);
                }
                ets_printf("gw:   interval histogram (peaks):\n");
                const uint32_t floor_count = st.transitions / 500;
                for (int k = 0; k < HIST_BUCKETS; k++) {
                    if (st.hist[k] > floor_count) {
                        ets_printf("gw:     %u.%02u us  %u\n",
                                   (k * 25) / 100, (k * 25) % 100,
                                   (unsigned)st.hist[k]);
                    }
                }
            }
            // Both passes read out of s_rx, so they have to finish before the
            // next command resets it.
            ok = track_decode(p, n, st, out, verbose, cell_bias);
        }
    }
    // The status has to be collected whatever happened, or the next command
    // reads it instead of its own acknowledgement.
    gw_cmd2(CMD_GETFLUXSTATUS);
    return ok;
}

// Progress goes two places: the serial log, in full, and whatever the caller
// wants to put it on. The menu passes a function that draws it on the panel,
// so a two-minute read is not two minutes of a blank screen.
typedef void (*gw_progress_fn)(const char *text);
static gw_progress_fn s_prog = nullptr;

static void prog(const char *fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ets_printf("gw: %s\n", buf);
    if (s_prog) {
        s_prog(buf);
    }
}

// ---- NFD r0 ----------------------------------------------------------------
// The output format. A raw image holds nothing but the bytes, so a sector that
// would not read comes out looking exactly like one that read as zeros; NFD
// carries a status per sector, which is the difference between an image that
// is honest about its gaps and one that is not.
//
// The layout is fixed by the struct the emulator reads it with
// (diskimage/fd/fdd_head_nfd.h, NFD_FILE_HEAD):
//
//   0x000  "T98FDDIMAGE.R0"                15 bytes plus one reserved
//   0x010  comment                         0x100 bytes, NUL terminated
//   0x110  dwHeadSize                      u32, always the value below
//   0x114  flProtect                       nonzero = write protected
//   0x115  byHead                          number of heads
//   0x120  si[163][26]                     sector IDs, 16 bytes each
//   ...    0x10 reserved, then the sector data in si order
//
// and each sector ID is
//
//   C H R N  flMFM flDDAM byStatus byST0 byST1 byST2 byPDA  5 reserved
//
// with C = 0xFF meaning there is no sector there. Track number is
// cylinder * 2 + head (nfd_track_index() in fdd_nfd.c), and the data follows
// in exactly that order - which is the same order a raw image uses, so the
// only thing that changes about the writing below is that a header goes in
// front of it.

#define NFD_TRKMAX     163
#define NFD_SECMAX     26
#define NFD_HDRSIZE    (288 + 16 * NFD_TRKMAX * NFD_SECMAX + 0x10)
#define NFD_SI_BASE    0x120

// FDD BIOS result codes, as the emulator hands them back to the guest
// (fddlasterror = byStatus in fdd_nfd.c).
#define NFD_OK         0x00
#define NFD_CRC_ERROR  0xa0
#define NFD_NO_DATA    0xe0

static inline uint8_t *nfd_entry(uint8_t *h, int trk, int r) {
    return h + NFD_SI_BASE + ((size_t)trk * NFD_SECMAX + (r - 1)) * 16;
}

static void nfd_header_init(uint8_t *h, int cyls, int spt, int ncode) {
    memset(h, 0, NFD_HDRSIZE);
    memcpy(h, "T98FDDIMAGE.R0", 14);

    // The clock is set by now if it is ever going to be, so the image can say
    // when it was taken - which is the only place a date is needed, the file
    // itself being named by a counter.
    const time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char *comment = (char *)h + 0x10;
    if (tmv.tm_year > 100) {
        snprintf(comment, 0x100, "np2 espresso Tab5  %04d-%02d-%02d %02d:%02d",
                 tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                 tmv.tm_hour, tmv.tm_min);
    } else {
        snprintf(comment, 0x100, "np2 espresso Tab5");
    }

    h[0x110] = (uint8_t)(NFD_HDRSIZE & 0xff);
    h[0x111] = (uint8_t)((NFD_HDRSIZE >> 8) & 0xff);
    h[0x112] = (uint8_t)((NFD_HDRSIZE >> 16) & 0xff);
    h[0x113] = (uint8_t)((NFD_HDRSIZE >> 24) & 0xff);
    h[0x114] = 0;          // not write protected
    h[0x115] = 2;          // heads

    // Everything is "no sector" until a real one is put in its place.
    for (int i = 0; i < NFD_TRKMAX * NFD_SECMAX; i++) {
        h[NFD_SI_BASE + (size_t)i * 16] = 0xff;
    }

    // The physical drive address, chosen the same way the emulator chooses it
    // when it formats a disk of its own (fdd_nfd.c).
    uint8_t pda = 0x90;                       // 2HD, 1.2MB
    if (ncode == 2) {
        if (spt < 10) {
            pda = 0x10;
        } else if (spt > 16) {
            pda = 0x30;
        }
    }
    for (int cyl = 0; cyl < cyls; cyl++) {
        for (int head = 0; head < 2; head++) {
            const int trk = cyl * 2 + head;
            for (int r = 1; r <= spt; r++) {
                uint8_t *e = nfd_entry(h, trk, r);
                e[0] = (uint8_t)cyl;
                e[1] = (uint8_t)head;
                e[2] = (uint8_t)r;
                e[3] = (uint8_t)ncode;
                e[4] = 1;                     // MFM
                e[5] = 0;                     // not a deleted-data mark
                e[6] = NFD_OK;
                e[10] = pda;
            }
        }
    }
}

// Records how a sector actually read. grade is the same 0/1/2 the dump loop
// uses: never seen, decoded but the CRC failed, clean.
static void nfd_mark(uint8_t *h, int trk, int r, int grade, bool deleted) {
    uint8_t *e = nfd_entry(h, trk, r);
    if (grade == 2) {
        e[5] = deleted ? 1 : 0;
        return;
    }
    if (grade == 1) {
        e[6] = NFD_CRC_ERROR;
        e[8] = 0x20;                          // ST1 DE  - data error
        e[9] = 0x20;                          // ST2 DD  - in the data field
    } else {
        e[6] = NFD_NO_DATA;
        e[7] = 0x40;                          // ST0 abnormal termination
        e[8] = 0x01;                          // ST1 MA  - missing address mark
    }
}

// ---- whole disk ------------------------------------------------------------
#define DUMP_REVS    3          // two complete revolutions per capture
#define DUMP_REVS_SLOW 5        // four, once a track has proved difficult
#define DUMP_RETRIES 10         // captures per track before giving up on it
#define DUMP_DIR     "/sd"

// How many cylinders a disk of this shape has. The emulator recognises a raw
// image purely by its size (fdd_set_xdf), so this has to be right or the
// result will not mount - which is also why the cylinder count is not simply
// probed until the reads stop working.
static int cyls_for(int spt, int n, unsigned kbps) {
    if (kbps > 375 && spt == 8 && n == 3) {
        return 77;              // PC-98 2HD 1.25MB - the .HDM this mostly means
    }
    return 80;                  // 2HC, 1.44MB and every 2DD layout
}

// Dumps are numbered rather than dated: the number cannot collide and needs no
// clock. The date goes in two places that do not need it to be in the name -
// the file's own timestamp, and the comment inside the image.
static bool next_dump_name(char *out, size_t cap) {
    for (int i = 1; i <= 9999; i++) {
        snprintf(out, cap, "%s/FD%04d.NFD", DUMP_DIR, i);
        struct stat sb;
        if (stat(out, &sb) != 0) {
            return true;
        }
    }
    return false;
}

extern "C" bool gw_dump_disk(gw_progress_fn progress) {
    s_prog = progress;
    if (!s_dev) {
        prog("no Greaseweazle on the USB port");
        return false;
    }
    if (!gw_cmd3(CMD_SETBUSTYPE, BUS_IBMPC) || !gw_set_delays() ||
        !gw_cmd3(CMD_SELECT, 0) || !gw_motor(0, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));            // the command above did the waiting

    track_out tr = {};
    tr.data = (uint8_t *)heap_caps_malloc((size_t)MAX_SECT * SECT_STRIDE,
                                          MALLOC_CAP_SPIRAM);
    uint8_t *tbuf = (uint8_t *)heap_caps_malloc((size_t)MAX_SECT * SECT_STRIDE,
                                                MALLOC_CAP_SPIRAM);
    uint8_t *hdr = (uint8_t *)heap_caps_malloc(NFD_HDRSIZE, MALLOC_CAP_SPIRAM);
    FILE *f = nullptr;
    bool ok = (tr.data && tbuf && hdr);

    // Cylinder zero decides the shape of the whole disk.
    int spt = 0, ncode = 0, cyls = 0;
    size_t seclen = 0;
    if (ok) {
        ok = track_read(0, 0, DUMP_REVS, tr, true);
        if (!ok) {
            ets_printf("gw: cylinder 0 is unreadable - is there a disk in the drive?\n");
        }
    }
    if (ok) {
        for (int k = 0; k < tr.nsect; k++) {
            if (tr.hit[k].r > spt) {
                spt = tr.hit[k].r;
            }
        }
        ncode = tr.hit[0].n & 7;
        seclen = (size_t)128 << ncode;
        cyls = cyls_for(spt, ncode, tr.kbps);
        ok = (spt > 0 && seclen <= SECT_STRIDE);
        ets_printf("gw: %d cyl x 2 head x %d sect x %u bytes = %u bytes (%s, %u.%u rpm)\n",
                   cyls, spt, (unsigned)seclen,
                   (unsigned)((size_t)cyls * 2 * spt * seclen),
                   (tr.kbps > 375) ? "2HD" : "2DD", tr.rpm10 / 10, tr.rpm10 % 10);
    }

    char path[64] = {0};
    if (ok) {
        ok = next_dump_name(path, sizeof(path));
        if (ok) {
            f = fopen(path, "wb");
            ok = (f != nullptr);
        }
        if (!ok) {
            ets_printf("gw: could not create a file on the SD card\n");
        } else {
            ets_printf("gw: writing %s\n", path);
            // Written now so the sector data lands at the right offset, and
            // again at the end once every sector's status is known.
            nfd_header_init(hdr, cyls, spt, ncode);
            ok = fwrite(hdr, 1, NFD_HDRSIZE, f) == NFD_HDRSIZE;
            if (!ok) {
                ets_printf("gw: could not write the header - SD card full?\n");
            }
        }
    }

    int bad_total = 0, missing_total = 0, odd_tracks = 0, clean_total = 0;
    const int want_total = cyls * 2 * spt;
    for (int cyl = 0; ok && cyl < cyls; cyl++) {
        for (int head = 0; ok && head < 2; head++) {
            // 0 = never seen, 1 = decoded but the CRC failed, 2 = clean.
            uint8_t got[MAX_SECT] = {0};
            bool odd = false;
            memset(tbuf, 0, (size_t)MAX_SECT * SECT_STRIDE);

            for (int attempt = 0; attempt < DUMP_RETRIES; attempt++) {
                if (attempt && (attempt % 2) == 0) {
                    // Re-reading a marginal track without moving the head just
                    // reads the same marginal flux again. Stepping away and
                    // back re-seats it, which is often all a sector that keeps
                    // failing its CRC needs.
                    gw_cmd3(CMD_SEEK, (uint8_t)((cyl > 2) ? cyl - 2 : cyl + 2));
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                // A sector right on the edge of readable can decode with a
                // clock a fraction off the measured one and not with the
                // measured one itself. Each attempt is a fresh capture
                // regardless, so trying a different hypothesis each time
                // costs nothing and picks up sectors that otherwise stick.
                static const float bias[DUMP_RETRIES] = {
                    1.00f, 1.00f, 0.98f, 1.02f, 1.00f,
                    0.97f, 1.03f, 0.99f, 1.01f, 1.00f,
                };
                // Capture more of the disk once a track has given trouble:
                // every extra revolution is another chance at the sector
                // that failed, and by then the extra second is worth it.
                const int revs = (attempt < 2) ? DUMP_REVS : DUMP_REVS_SLOW;
                if (!track_read(cyl, head, revs, tr, false, bias[attempt])) {
                    continue;
                }
                for (int k = 0; k < tr.nsect; k++) {
                    const int r = tr.hit[k].r;
                    if (r < 1 || r > spt || (tr.hit[k].n & 7) != ncode) {
                        // A sector the disk has but this geometry has no room
                        // for. A plain DOS disk never has one; noting it is
                        // what stops an odd disk being written out silently as
                        // though it were fine.
                        odd = true;
                        continue;
                    }
                    const uint8_t grade = tr.hit[k].data_ok ? 2 : 1;
                    if (grade <= got[r - 1]) {
                        continue;      // already have this one, as good or better
                    }
                    memcpy(tbuf + (size_t)(r - 1) * SECT_STRIDE,
                           tr.data + (size_t)k * SECT_STRIDE, seclen);
                    got[r - 1] = grade;
                }
                int clean = 0;
                for (int r = 0; r < spt; r++) {
                    if (got[r] == 2) {
                        clean++;
                    }
                }
                if (clean == spt) {
                    break;
                }
            }

            int clean = 0, bad = 0, missing = 0;
            for (int r = 0; r < spt; r++) {
                if (got[r] == 2) {
                    clean++;
                } else if (got[r] == 1) {
                    bad++;
                } else {
                    missing++;
                }
                nfd_mark(hdr, cyl * 2 + head, r + 1, got[r], false);
                // A sector that failed its CRC is still written out: a wrong
                // byte somewhere beats a hole where the data should be, and
                // the count below says how much of the image to distrust.
                if (fwrite(tbuf + (size_t)r * SECT_STRIDE, 1, seclen, f) != seclen) {
                    ets_printf("gw: write failed - SD card full?\n");
                    ok = false;
                    break;
                }
            }
            bad_total += bad;
            missing_total += missing;
            if (odd) {
                odd_tracks++;
            }
            clean_total += clean;
            if (clean == spt) {
                prog("C%02d H%d   %d/%d sectors", cyl, head, clean_total, want_total);
            } else {
                prog("C%02d H%d   %d/%d   (%d bad crc, %d missing)",
                     cyl, head, clean_total, want_total, bad, missing);
            }
        }
    }

    if (f) {
        // Now that every sector's fate is known, put the table back with the
        // statuses filled in.
        if (ok && (fseek(f, 0, SEEK_SET) != 0 ||
                   fwrite(hdr, 1, NFD_HDRSIZE, f) != NFD_HDRSIZE)) {
            ets_printf("gw: could not rewrite the header\n");
            ok = false;
        }
        fclose(f);
    }
    gw_motor(0, false);
    gw_cmd2(CMD_DESELECT);
    free(tr.data);
    free(tbuf);
    free(hdr);

    if (ok) {
        const unsigned total = (unsigned)(NFD_HDRSIZE + (size_t)cyls * 2 * spt * seclen);
        if (odd_tracks) {
            // This only handles plain DOS disks. Saying so plainly beats
            // handing back an image that looks complete: the sectors that did
            // not fit are simply gone from it.
            ets_printf("gw: WARNING - %d tracks carry sectors this format cannot hold,\n",
                       odd_tracks);
            ets_printf("gw:   so they were left out. This is not a plain DOS disk and\n");
            ets_printf("gw:   %s is not a faithful copy of it.\n", path);
        }
        const char *name = strrchr(path, '/');
        name = name ? name + 1 : path;
        if (bad_total || missing_total) {
            prog("%s  %u bytes, %d bad, %d missing%s", name, total,
                 bad_total, missing_total, odd_tracks ? "  NOT PLAIN DOS" : "");
        } else if (odd_tracks) {
            prog("%s  %u bytes  NOT A PLAIN DOS DISK", name, total);
        } else {
            prog("%s  %u bytes, every sector clean", name, total);
        }
    }
    s_prog = nullptr;
    return ok && bad_total == 0 && missing_total == 0 && odd_tracks == 0;
}

// ---- entry -----------------------------------------------------------------
// Called once the USB host stack is up (usb_kbd_init installs it). Returns
// false when no Greaseweazle is plugged in, which is the normal case.
// The driver tells us when the device goes away; without this a Greaseweazle
// that is unplugged and plugged back in would leave a stale handle behind and
// every command would fail until the next reboot.
static void gw_event_cb(const cdc_acm_host_dev_event_data_t *e, void *arg) {
    (void)arg;
    if (e->type == CDC_ACM_HOST_ERROR) {
        ets_printf("gw: transfer error %d\n", e->data.error);
    } else if (e->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        s_dev = nullptr;
        cdc_acm_host_close(e->data.cdc_hdl);
        ets_printf("gw: Greaseweazle unplugged\n");
    }
}

extern "C" bool gw_probe(void) {
    if (s_dev) {
        return s_freq != 0;                  // already open from an earlier call
    }
    if (!s_rx) {
        s_rx = (uint8_t *)heap_caps_malloc(GW_RX_CAP, MALLOC_CAP_SPIRAM);
        s_rx_sig = xSemaphoreCreateBinary();
        if (!s_rx || !s_rx_sig) {
            ets_printf("gw: out of memory\n");
            return false;
        }
    }
    static bool installed = false;
    if (!installed) {
        const cdc_acm_host_driver_config_t drv = {
            .driver_task_stack_size = 4096,
            .driver_task_priority = 5,
            .xCoreID = 0,
            .new_dev_cb = nullptr,
        };
        const esp_err_t e = cdc_acm_host_install(&drv);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
            ets_printf("gw: cdc_acm_host_install failed: %s\n", esp_err_to_name(e));
            return false;
        }
        installed = true;
    }

    cdc_acm_host_device_config_t cfg = {};
    cfg.connection_timeout_ms = 1000;
    cfg.out_buffer_size = 512;
    cfg.in_buffer_size = 4096;       // fewer, larger callbacks during a read
    cfg.event_cb = gw_event_cb;
    cfg.data_cb = rx_cb;
    cfg.user_arg = nullptr;

    const esp_err_t e = cdc_acm_host_open(GW_VID, GW_PID, 0, &cfg, &s_dev);
    if (e != ESP_OK) {
        ets_printf("gw: no Greaseweazle on the USB port (%04x:%04x)\n", GW_VID, GW_PID);
        s_dev = nullptr;
        return false;
    }
    ets_printf("gw: Greaseweazle found\n");

    // The device ignores the line coding, but some hosts will not open the pipe
    // without one being set.
    cdc_acm_line_coding_t lc = {115200, 0, 0, 8};
    cdc_acm_host_line_coding_set(s_dev, &lc);

    const bool ok = gw_get_info();
    if (!ok) {
        ets_printf("gw: the device did not answer GetInfo\n");
    }
    return ok;
}
