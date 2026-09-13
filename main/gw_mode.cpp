// Greaseweazle over the USB-A port: read the real floppy in the drive.
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
// recover the bitstream and find the sectors. Nothing is written to a file -
// fdd_gw_live.cpp serves the sectors straight to the emulator, so a disk is
// played from the original rather than from a copy of it.

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "esp_timer.h"
#include "gw_live.h"

extern "C" int ets_printf(const char *fmt, ...);
extern "C" bool gw_probe(void);        // defined at the end of this file
static bool gw_probe_unit(int unit);
static bool unit_select(int unit);
static const char *serial_of(uint8_t addr);
static uint8_t port_of(uint8_t addr);

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

// Two Greaseweazles, one per floppy drive. A PC-98 has two, and an installer
// that wants disk B in drive 2 cannot be answered with one. They are told apart
// by USB address: every Greaseweazle carries the same VID and PID, which is
// exactly the case cdc_acm_host_open_config_t::dev_addr exists for.
#define GW_UNITS 2

struct gw_dev {
    cdc_acm_dev_hdl_t dev;
    uint8_t          *rx;
    volatile size_t   rx_len;             // bytes written by the USB callback
    size_t            rx_pos;             // bytes consumed by the reader
    volatile bool     rx_ovf;
    SemaphoreHandle_t rx_sig;
    SemaphoreHandle_t rx_lock;            // held while rx_len is changed
    uint32_t          freq;               // sample ticks per second
    uint8_t           addr;               // USB device address, 0 = not found
};

static gw_dev s_gw[GW_UNITS];

// The unit every helper below works on. The emulator serialises floppy access
// through the FDC, so one at a time is all that is ever needed, and a current
// unit keeps the command helpers as short as they were.
static gw_dev *g = &s_gw[0];

// ---- transport -------------------------------------------------------------
// Append-only: the callback fills the buffer and advances rx_len behind the
// data it has already copied, the reader advances rx_pos behind that. Reading
// needs no lock - the writer only ever grows rx_len, and never publishes bytes
// it has not written yet.
//
// Rewinding does. rx_reset() puts rx_len back to zero from the emulator task
// while this callback runs in the CDC-ACM driver's, and both are a
// read-modify-write of the same word. Losing that race leaves rx_len holding
// the callback's total with rx_pos back at zero, so the next acknowledgement
// is read from the middle of whatever the buffer still held. rx_lock is held
// across the copy on this side and across the rewind on the other.
static bool rx_cb(const uint8_t *data, size_t len, void *arg) {
    // Whose data this is comes from the callback argument, not from the
    // current unit: with two devices open, the other one's driver task can be
    // delivering while this one is being read.
    gw_dev *d = (gw_dev *)arg;
    xSemaphoreTake(d->rx_lock, portMAX_DELAY);
    const size_t room = GW_RX_CAP - d->rx_len;
    if (len > room) {
        d->rx_ovf = true;
        len = room;
    }
    memcpy(d->rx + d->rx_len, data, len);
    d->rx_len = d->rx_len + len;
    xSemaphoreGive(d->rx_lock);
    xSemaphoreGive(d->rx_sig);
    return true;   // we took the buffer
}

static void rx_reset(void) {
    xSemaphoreTake(g->rx_lock, portMAX_DELAY);
    g->rx_len = 0;
    g->rx_pos = 0;
    g->rx_ovf = false;
    xSemaphoreGive(g->rx_lock);
    xSemaphoreTake(g->rx_sig, 0);
}

// Wait for the device to stop talking, throwing away whatever it says.
//
// READFLUX has no abort: it streams until it has given the revolutions it was
// asked for. Giving up on the terminator therefore leaves the rest of the
// stream to arrive in the next command's answer, and every command after that
// reads someone else's. Quiet is the signal that it has finished - flux comes
// in a steady rush, a callback every few tens of milliseconds, so a fifth of a
// second of silence is not a pause in it.
//
// The rewind inside the loop is what keeps a stream longer than the buffer
// from filling it and going quiet for the wrong reason.
static void rx_drain(int quiet_ms, int cap_ms) {
    const TickType_t hard = xTaskGetTickCount() + pdMS_TO_TICKS(cap_ms);
    TickType_t last = xTaskGetTickCount();
    size_t seen = g->rx_len;

    for (;;) {
        xSemaphoreTake(g->rx_sig, pdMS_TO_TICKS(20));
        const TickType_t now = xTaskGetTickCount();
        const size_t have = g->rx_len;

        if (have != seen) {
            last = now;
            seen = have;
            if (have > GW_RX_CAP / 2) {
                rx_reset();
                seen = 0;
            }
        } else if ((now - last) >= pdMS_TO_TICKS(quiet_ms)) {
            break;
        }
        if (now >= hard) {
            ets_printf("gw: the device is still streaming after %d ms\n", cap_ms);
            break;
        }
    }
    rx_reset();
}

// Wait for `want` more bytes and take them. The device answers a command within
// a millisecond or two; a read that times out means the command was not
// understood, which is worth saying rather than hanging.
static bool rx_take(uint8_t *out, size_t want, int timeout_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        if (g->rx_len - g->rx_pos >= want) {
            memcpy(out, g->rx + g->rx_pos, want);
            g->rx_pos += want;
            return true;
        }
        if (xTaskGetTickCount() >= deadline) {
            return false;
        }
        xSemaphoreTake(g->rx_sig, pdMS_TO_TICKS(20));
    }
}

// Scan forward for the stream terminator, waiting for more data as needed.
static bool rx_wait_zero(size_t *end, int timeout_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    size_t scan = g->rx_pos;
    for (;;) {
        const size_t have = g->rx_len;
        while (scan < have) {
            if (g->rx[scan] == 0) {
                *end = scan;
                return true;
            }
            scan++;
        }
        if (g->rx_ovf) {
            ets_printf("gw: receive buffer overflowed\n");
            return false;
        }
        if (xTaskGetTickCount() >= deadline) {
            return false;
        }
        xSemaphoreTake(g->rx_sig, pdMS_TO_TICKS(20));
    }
}

// How many stale answers to step over before giving up, and how long to wait
// for each. One is the usual case - a single command timed out - and the cap is
// there so a device sending something that is not an answer at all cannot hold
// the emulator up.
#define GW_RESYNC_MAX 8
#define GW_RESYNC_MS  200

// Step forward until the echoed opcode is the one that was just sent.
//
// A command whose answer arrived too late is not lost: the device answers
// everything it is sent, in order, so that answer turns up in the next
// command's slot and every read after it is one command behind. Two bytes per
// answer means the stream can be walked back into step, and what is walked over
// belongs to commands that have already failed and been reported.
//
// Without this the link stays crossed until the board is power-cycled, which is
// what the log showed: "got 02, sent 0e" followed by "got 0e, sent 02", over
// and over, from one seek that took longer than a second.
static bool rx_resync(uint8_t want, uint8_t *ack) {
    for (int stale = 1; stale <= GW_RESYNC_MAX; stale++) {
        if (!rx_take(ack, 2, GW_RESYNC_MS)) {
            return false;
        }
        if (ack[0] == want) {
            ets_printf("gw: back in step, %d stale repl%s skipped\n",
                       stale, stale == 1 ? "y" : "ies");
            return true;
        }
    }
    return false;
}

// Send a command and check the acknowledgement. Every Greaseweazle command
// answers with its own opcode and a status byte, so a mismatched opcode means
// the stream has lost sync rather than that the command failed.
static bool gw_cmd(const uint8_t *cmd, size_t len, int timeout_ms = 1000) {
    // Anything sitting in the buffer now is the tail of an exchange that has
    // already been given up on. Dropping it before sending saves a step of the
    // resync below; it cannot prevent the crossing, because the answer that
    // causes it has not arrived yet.
    rx_reset();
    if (cdc_acm_host_data_tx_blocking(g->dev, cmd, len, 1000) != ESP_OK) {
        ets_printf("gw: tx failed (cmd %u)\n", cmd[0]);
        return false;
    }
    uint8_t ack[2];
    if (!rx_take(ack, sizeof(ack), timeout_ms)) {
        ets_printf("gw: no reply to cmd %u\n", cmd[0]);
        return false;
    }
    if (ack[0] != cmd[0] && !rx_resync(cmd[0], ack)) {
        ets_printf("gw: out of sync (got %02x, sent %02x) and could not recover\n",
                   ack[0], cmd[0]);
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
    g->freq = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
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
               (unsigned)g->freq,
               g->freq ? (unsigned)(1000000000000ULL / g->freq) : 0u);
    ets_printf("gw: mcu %u MHz, %u KB SRAM, %u KB USB buffer\n",
               mcu_mhz, mcu_sram_kb, usb_buf_kb);
    return g->freq != 0;
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
        30000,   // watchdog, ms - long gaps between reads are normal here
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
                        flux_sink_t sink = nullptr, void *arg = nullptr,
                        bool cue_at_index = false) {
    const uint32_t bucket = g->freq / 4000000;    // ticks per 0.25us
    memset(&st, 0, sizeof st);
    if (bucket == 0) {
        return false;
    }
    size_t i = 0;
    uint32_t ticks = 0;              // accumulated since the last transition
    int64_t since_index = 0;         // may go negative, exactly as upstream
    bool sink_cued = !cue_at_index;
    bool first_after_index = false;
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
                if (cue_at_index && !sink_cued) {
                    sink_cued = true;
                    first_after_index = true;
                }
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
            if (sink && sink_cued) {
                if (first_after_index) {
                    const int64_t clipped = since_index + ticks;
                    if (clipped > 0) {
                        sink((uint32_t)clipped, arg);
                    }
                    first_after_index = false;
                } else {
                    sink(ticks, arg);
                }
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
#define SECT_STRIDE GW_SECT_MAX_LEN   // 2048: N=4 sectors turn up on
                                      // protected tracks and must fit

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
    const uint32_t bucket = g->freq / 4000000;
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
    bool     fm;         // found by the single-density pass
    int      seen;
};

// All four ID bytes, because that is what the controller compares. Two
// sectors on one track can share a number and differ in size - it is a common
// way to protect a disk - and folding them together loses the one the guest
// is looking for.
static int sect_find(const sect_hit *v, int cnt, uint8_t c, uint8_t h,
                    uint8_t r, uint8_t n) {
    for (int k = 0; k < cnt; k++) {
        if (v[k].c == c && v[k].h == h && v[k].r == r && v[k].n == n) {
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
            int k = sect_find(out, cnt, id[0], id[1], id[2], id[3]);
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
            // Deliberately NOT skipping to the end of the data field.
            //
            // The length read above comes from the sector's own N, and a
            // protected track is exactly where that number cannot be trusted:
            // declaring a sector larger than it really is makes an ordinary
            // read run off the end and fail, which is the check. Skipping by
            // the declared length then steps over the address mark of whatever
            // follows, and that sector vanishes from the track - which is what
            // was happening here, with R1 disappearing behind a 2048-byte
            // R240 and the sector count changing from one capture to the next.
            //
            // A real FDC never skips; it hunts for the next address mark
            // continuously. So does this now. Scanning back over the data
            // costs a little time and cannot produce a false sector: the sync
            // pattern is an A1 with a missing clock bit, which legal MFM data
            // cannot contain, and anything that did slip through would still
            // have to pass the ID CRC.
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

// Single density. See the note at the top of the FM patch: same clock/data
// interleave as MFM, different address marks, no A1 preamble in the CRC, and
// bit cells twice as long - which is why this runs over its own pass of the
// flux rather than the one MFM used.
#define FM_AM_ID      0xf57e
#define FM_AM_DATA    0xf56f
#define FM_AM_DELETED 0xf56a

static int fm_scan(uint32_t nbits, sect_hit *out, int max_out,
                   uint8_t *data, size_t dcap) {
    int cnt = 0;
    int pending = -1;
    uint32_t sr = 0;


    for (uint32_t i = 0; i < nbits; i++) {
        sr = (sr << 1) | s_bits[i];
        const uint16_t w = (uint16_t)sr;

        if (w == FM_AM_ID) {
            uint32_t p = i + 1;
            uint8_t id[6];
            for (int k = 0; k < 6; k++) {
                id[k] = mfm_byte(&p, nbits);
            }
            const uint8_t mark = 0xfe;
            uint16_t crc = crc16_ccitt(&mark, 1, 0xffff);
            crc = crc16_ccitt(id, 6, crc);
            if (crc != 0) {
                pending = -1;
                continue;
            }
            int k = sect_find(out, cnt, id[0], id[1], id[2], id[3]);
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
                out[k].fm = true;
            }
            out[k].id_ok = true;
            out[k].seen++;
            pending = k;

        } else if (w == FM_AM_DATA || w == FM_AM_DELETED) {
            if (pending < 0) {
                continue;
            }
            const int slot = pending;
            pending = -1;
            if (out[slot].data_ok) {
                continue;
            }
            const uint32_t len = 128u << (out[slot].n & 7);
            uint8_t *dst = (data && len <= SECT_STRIDE &&
                            (size_t)(slot + 1) * SECT_STRIDE <= dcap)
                           ? data + (size_t)slot * SECT_STRIDE : nullptr;
            const uint8_t mark = (w == FM_AM_DATA) ? 0xfb : 0xf8;
            uint32_t p = i + 1;
            uint16_t crc = crc16_ccitt(&mark, 1, 0xffff);
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
            out[slot].deleted = (w == FM_AM_DELETED);
            i = p - 1;
        }
    }
    return cnt;
}

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
    out.kbps = (unsigned)((float)g->freq / (2.0f * out.cell) / 1000.0f);
    if (verbose) {
        const unsigned cell_ns = (unsigned)(out.cell * 1000000000.0f / (float)g->freq);
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
    if (!decode_flux(stream, n, ignored, pll_sink, &pll, true)) {
        return false;
    }
    if (out.data) {
        memset(out.data, 0, (size_t)MAX_SECT * SECT_STRIDE);
    }
    out.nsect = mfm_scan(pll.nbits, out.hit, MAX_SECT,
                         out.data, (size_t)MAX_SECT * SECT_STRIDE);

    // Single density comes in two shapes on a 2HD disk: written at half the
    // data rate, where the cells are twice as long, and written at the full
    // rate, where they are the same length as the MFM ones around them. The
    // second kind is already sitting in the bitstream that has just been
    // recovered, so look there first - it costs nothing.
    if (out.nsect < MAX_SECT) {
        const int base = out.nsect;
        const int got = fm_scan(pll.nbits, out.hit + base, MAX_SECT - base,
                                out.data ? out.data + (size_t)base * SECT_STRIDE : nullptr,
                                (size_t)(MAX_SECT - base) * SECT_STRIDE);
        if (got > 0) {
            ets_printf("gw:   %d single-density sector%s at the MFM cell time\n",
                       got, got == 1 ? "" : "s");
        }
        out.nsect += got;
    }

    // ...and again at twice the cell time, for the half-rate kind. A track with
    // neither costs one more pass over the flux and finds nothing, which is the
    // usual case and cheap beside the half-second it took to capture.
    if (out.nsect < MAX_SECT) {
        pll_state fmpll = {out.cell * 2.0f, out.cell * 2.0f, 0.0f, 0};
        flux_stats ignored2;
        if (decode_flux(stream, n, ignored2, pll_sink, &fmpll, true)) {
            const int base = out.nsect;
            const int got = fm_scan(fmpll.nbits, out.hit + base, MAX_SECT - base,
                                    out.data ? out.data + (size_t)base * SECT_STRIDE : nullptr,
                                    (size_t)(MAX_SECT - base) * SECT_STRIDE);
            if (got > 0) {
                ets_printf("gw:   %d single-density sector%s at half the data rate\n",
                           got, got == 1 ? "" : "s");
            }
            out.nsect += got;
        }
    }

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
        // Not lost - still coming. Let it finish before anything else is
        // said, or the rest of it is read as the next command's answer.
        ets_printf("gw: flux stream did not terminate - waiting for the device\n");
        rx_drain(200, 3000);
    } else {
        const uint8_t *p = g->rx + g->rx_pos;
        const size_t n = end - g->rx_pos;
        g->rx_pos = end + 1;

        flux_stats st;
        ok = decode_flux(p, n, st);
        if (!ok) {
            ets_printf("gw: could not decode the flux stream\n");
        } else {
            out.rpm10 = (st.nindex > 1 && st.index_ticks[1])
                        ? (unsigned)((uint64_t)600 * g->freq / st.index_ticks[1]) : 0;
            if (verbose) {
                ets_printf("gw: cyl %d head %d: %u flux bytes, %u transitions, %d index\n",
                           cyl, head, (unsigned)n, (unsigned)st.transitions, st.nindex);
                for (int k = 1; k < st.nindex && k < 8; k++) {
                    const uint32_t t = st.index_ticks[k];
                    if (!t) {
                        continue;
                    }
                    const unsigned ms100 = (unsigned)((uint64_t)t * 100000ULL / g->freq);
                    ets_printf("gw:   rev %d: %u.%02u ms  =  %u.%u rpm\n", k,
                               ms100 / 100, ms100 % 100,
                               (unsigned)((uint64_t)600 * g->freq / t) / 10,
                               (unsigned)((uint64_t)600 * g->freq / t) % 10);
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
            // Both passes read out of g->rx, so they have to finish before the
            // next command resets it.
            ok = track_decode(p, n, st, out, verbose, cell_bias);
        }
    }
    // The status has to be collected whatever happened, or the next command
    // reads it instead of its own acknowledgement.
    gw_cmd2(CMD_GETFLUXSTATUS);
    return ok;
}

// ---- live reading ----------------------------------------------------------
// The same capture and decode the dump uses, but handing the sectors back to
// the caller instead of writing them to a file. fdd_gw_live.cpp serves the
// emulator's floppy accesses out of this, so a disk never has to be turned into
// an image - and a protected one keeps the sectors an image cannot hold.

static track_out s_live[GW_UNITS];   // one track's worth per unit
static bool s_live_on[GW_UNITS];     // ...and whether a disk is mounted on it
static bool s_spun[GW_UNITS];        // ...and whether its spindle is turning
static int64_t s_used_us[GW_UNITS];  // ...and when the machine last read from it
static uint32_t s_rests[GW_UNITS];   // ...and how many times it has been let stop

// How long the spindle keeps turning after the last read. Long enough that a
// game loading track after track never spins down between them, short enough
// that a machine sitting at a prompt is not wearing the disk while it waits.
#define GW_IDLE_HOLD_US 10000000
#define GW_SPINUP_MS    2000        // a 5.25-inch spindle, measured

// Select the unit every helper below works on. Out of range, or a unit with no
// Greaseweazle behind it, and nothing is selected.
static bool unit_select(int unit) {
    if (unit < 0 || unit >= GW_UNITS) {
        return false;
    }
    g = &s_gw[unit];
    return true;
}

extern "C" bool gw_live_begin(int unit) {
    if (!unit_select(unit)) {
        return false;
    }
    if (!g->dev && !gw_probe_unit(unit)) {
        return false;
    }
    track_out &live = s_live[unit];
    if (!live.data) {
        live.data = (uint8_t *)heap_caps_malloc((size_t)MAX_SECT * SECT_STRIDE,
                                                MALLOC_CAP_SPIRAM);
        if (!live.data) {
            ets_printf("gw: out of memory for the live track\n");
            return false;
        }
    }
    if (!gw_cmd3(CMD_SETBUSTYPE, BUS_IBMPC) || !gw_set_delays()) {
        return false;
    }
    if (!gw_cmd3(CMD_SELECT, 0) || !gw_motor(0, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(GW_SPINUP_MS));
    s_live_on[unit] = true;
    s_spun[unit] = true;
    s_used_us[unit] = esp_timer_get_time();
    ets_printf("gw: unit %d ready, hub port %u (USB address %u, serial %s)\n",
               unit, port_of(g->addr), g->addr, serial_of(g->addr));
    return true;
}

// Put the drive back under our control after it has slipped out of it.
//
// A live-mounted disk is read on demand, so the gap between one track and the
// next is however long the machine takes to ask - a second while a file
// manager redraws, a minute while nobody touches anything. The Greaseweazle
// does not hold a drive selected and its motor running across a gap like that,
// and the next command then comes back
//
//     gw: cmd 2 failed, status 7          (Seek, No_Unit)
//     gwfdd1: C0 H1 unreadable
//
// which the cache remembers as an empty track - so the file the machine was
// opening reads back as nothing at all. Reading straight through a disk never
// showed it: those tracks follow each other closely enough that the drive is
// never left alone.
//
// The whole opening sequence is repeated rather than just the select, because
// if the device went away and came back it has forgotten the bus type and the
// step delays too.
static bool gw_rearm(int unit) {
    if (!g->dev) {
        return false;
    }
    if (!gw_cmd3(CMD_SETBUSTYPE, BUS_IBMPC) || !gw_set_delays() ||
        !gw_cmd3(CMD_SELECT, 0) || !gw_motor(0, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(GW_SPINUP_MS));   // it has to be up to speed
    s_spun[unit] = true;
    ets_printf("gw: unit %d re-selected\n", unit);
    return true;
}

extern "C" void gw_live_end(int unit) {
    if (!unit_select(unit)) {
        return;
    }
    s_live_on[unit] = false;
    s_spun[unit] = false;
    if (g->dev) {
        gw_motor(0, false);
        gw_cmd2(CMD_DESELECT);
    }
}

// Bring the spindle up to speed, if it is not already. The wait is the whole
// point: asking for flux before the disk is turning returns no index at all,
// and the retries that follow cost far more than waiting once.
static bool gw_spin_up(int unit) {
    if (s_spun[unit]) {
        return true;
    }
    if (!gw_cmd3(CMD_SELECT, 0) || !gw_motor(0, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(GW_SPINUP_MS));
    s_spun[unit] = true;
    return true;
}

// Called from the emulator's own loop, which is also where reads come from, so
// there is no second thread to race against and no lock to take. It holds the
// motor on through a burst of reads, and lets it go once the machine has
// stopped asking - so the disk turns while it is being used and rests when it
// is not.
extern "C" void gw_live_keepalive(void) {
    static int64_t due_us = 0;
    const int64_t now = esp_timer_get_time();

    if (now < due_us) {
        return;
    }
    due_us = now + 3000000;         // comfortably inside the 30s watchdog

    for (int u = 0; u < GW_UNITS; u++) {
        if (!s_live_on[u] || !s_spun[u] || !unit_select(u) || !g->dev) {
            continue;
        }
        if (now - s_used_us[u] < GW_IDLE_HOLD_US) {
            gw_motor(0, true);          // still in use: keep it turning
        } else {
            gw_motor(0, false);
            gw_cmd2(CMD_DESELECT);
            s_spun[u] = false;
            s_rests[u]++;   // a stopped drive is a drive whose disk can change
            ets_printf("gw: unit %d idle - motor off\n", u);
        }
    }
}

extern "C" int gw_live_read_track(int unit, int cyl, int head, int revs,
                                  gw_sector_t *out, int max_out,
                                  uint8_t *data, size_t data_cap) {
    if (!unit_select(unit)) {
        return -1;
    }
    track_out &live = s_live[unit];
    if (!g->dev || !live.data || !out || !data) {
        return -1;
    }
    // The machine wants this drive: note it, so the keep-alive holds the motor
    // on through the burst of reads that usually follows, and spin the spindle
    // up first if it had been left to rest.
    s_used_us[unit] = esp_timer_get_time();
    if (!gw_spin_up(unit)) {
        return -1;
    }
    if (!track_read(cyl, head, revs, live, false)) {
        // Most likely the drive was let go while nothing was reading it. Take
        // it back and ask once more; a track that is genuinely unreadable will
        // fail the second time too and be reported as it was before.
        if (!gw_rearm(unit) || !track_read(cyl, head, revs, live, false)) {
            return -1;
        }
    }
    int n = 0;
    uint32_t off = 0;
    for (int k = 0; k < live.nsect && n < max_out; k++) {
        const sect_hit &h = live.hit[k];
        const uint32_t len = 128u << (h.n & 7);
        if (len > SECT_STRIDE || off + len > data_cap) {
            continue;                   // no room; better to drop than to lie
        }
        memcpy(data + off, live.data + (size_t)k * SECT_STRIDE, len);
        out[n].c = h.c;
        out[n].h = h.h;
        out[n].r = h.r;
        out[n].n = h.n;
        out[n].id_ok = h.id_ok ? 1 : 0;
        out[n].data_ok = h.data_ok ? 1 : 0;
        out[n].deleted = h.deleted ? 1 : 0;
        out[n].fm = h.fm ? 1 : 0;
        out[n].seen = (uint8_t)((h.seen > 255) ? 255 : h.seen);
        out[n].len = (uint16_t)len;
        out[n].off = off;
        off += len;
        n++;
    }
    return n;
}

// The serial the device reports, so the menu can name the drive it mounted.
// Empty when the unit was never opened.
extern "C" const char *gw_live_serial(int unit) {
    if (!unit_select(unit) || !g->dev) {
        return "";
    }
    return serial_of(g->addr);
}

// How many times this drive has been allowed to stop. It only goes up, and a
// change in it means the disk in the drive could have been swapped since the
// last read - which is the only moment it can have been.
extern "C" uint32_t gw_live_rest_count(int unit) {
    if (unit < 0 || unit >= GW_UNITS) {
        return 0;
    }
    return s_rests[unit];
}

// The hub socket it is plugged into - a thing you can follow a cable to, which
// is what the menu wants. 0 when the unit was never opened.
extern "C" int gw_live_port(int unit) {
    if (!unit_select(unit) || !g->dev) {
        return 0;
    }
    return (int)port_of(g->addr);
}

extern "C" int gw_live_count(void) {
    int n = 0;
    for (int i = 0; i < GW_UNITS; i++) {
        if (s_gw[i].addr) {
            n++;
        }
    }
    return n;
}

// ---- entry -----------------------------------------------------------------
// Called once the USB host stack is up (usb_kbd_init installs it). Returns
// false when no Greaseweazle is plugged in, which is the normal case.
// The driver tells us when the device goes away; without this a Greaseweazle
// that is unplugged and plugged back in would leave a stale handle behind and
// every command would fail until the next reboot.
static void gw_event_cb(const cdc_acm_host_dev_event_data_t *e, void *arg) {
    gw_dev *d = (gw_dev *)arg;
    if (e->type == CDC_ACM_HOST_ERROR) {
        ets_printf("gw: transfer error %d\n", e->data.error);
    } else if (e->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        if (d) {
            d->dev = nullptr;
            d->addr = 0;
        }
        cdc_acm_host_close(e->data.cdc_hdl);
        ets_printf("gw: Greaseweazle unplugged\n");
    }
}

// Every Greaseweazle answers to the same VID and PID, so a second one cannot be
// opened by asking for that pair again - the driver would hand back the first.
// The host stack offers each new device to this callback before anything opens
// it, which is where their USB addresses are collected. Opening then names an
// address, and the two drives stay apart.
static uint8_t s_found[GW_UNITS];
static char    s_serial[GW_UNITS][20];   // as reported by the device itself
static uint8_t s_port[GW_UNITS];         // which socket on the hub it is in
static int s_nfound = 0;

// The string descriptor is UTF-16LE and these serials are plain hex, so the
// low byte of each unit is the whole of it.
static void serial_text(const usb_str_desc_t *d, char *out, size_t cap) {
    size_t n = 0;
    if (d && d->bLength > 2) {
        const size_t chars = (size_t)(d->bLength - 2) / 2;
        for (size_t i = 0; i < chars && n + 1 < cap; i++) {
            const uint16_t c = d->wData[i];
            out[n++] = (c >= 0x20 && c < 0x7f) ? (char)c : '?';
        }
    }
    out[n] = 0;
}

// The serial of the device at this USB address, or "?" if it never said.
static const char *serial_of(uint8_t addr) {
    for (int i = 0; i < s_nfound; i++) {
        if (s_found[i] == addr) {
            return s_serial[i][0] ? s_serial[i] : "?";
        }
    }
    return "?";
}

// The hub socket the device at this address is plugged into. 0 when the host
// never said, which is what a device on the root port reports.
static uint8_t port_of(uint8_t addr) {
    for (int i = 0; i < s_nfound; i++) {
        if (s_found[i] == addr) {
            return s_port[i];
        }
    }
    return 0;
}

static void gw_new_dev_cb(usb_device_handle_t usb_dev) {
    const usb_device_desc_t *desc = nullptr;
    usb_device_info_t info = {};
    if (usb_host_get_device_descriptor(usb_dev, &desc) != ESP_OK || !desc) {
        return;
    }
    if (desc->idVendor != GW_VID || desc->idProduct != GW_PID) {
        return;
    }
    if (usb_host_device_info(usb_dev, &info) != ESP_OK) {
        return;
    }
    for (int i = 0; i < s_nfound; i++) {
        if (s_found[i] == info.dev_addr) {
            return;                       // already known
        }
    }
    if (s_nfound < GW_UNITS) {
        serial_text(info.str_desc_serial_num, s_serial[s_nfound], sizeof(s_serial[0]));
        s_found[s_nfound] = info.dev_addr;
        s_port[s_nfound] = info.parent.port_num;
        ets_printf("gw: Greaseweazle on hub port %u (USB address %u, serial %s)\n",
                   info.parent.port_num, info.dev_addr,
                   s_serial[s_nfound][0] ? s_serial[s_nfound] : "?");
        s_nfound++;
    }
}

static bool driver_up(void) {
    static bool installed = false;
    if (installed) {
        return true;
    }
    const cdc_acm_host_driver_config_t drv = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 0,
        .new_dev_cb = gw_new_dev_cb,
    };
    const esp_err_t e = cdc_acm_host_install(&drv);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ets_printf("gw: cdc_acm_host_install failed: %s\n", esp_err_to_name(e));
        return false;
    }
    installed = true;
    // The callback only fires for devices that arrive after this point, so
    // anything already plugged in has to be found the other way: open with any
    // address, note what turned up, and keep it.
    return true;
}

// Bring up one unit. Returns false when there is no Greaseweazle for it, which
// is the normal case for unit 1 on a one-drive setup.
static bool gw_probe_unit(int unit) {
    if (!unit_select(unit)) {
        return false;
    }
    if (g->dev) {
        return g->freq != 0;              // already open
    }
    if (!g->rx) {
        g->rx = (uint8_t *)heap_caps_malloc(GW_RX_CAP, MALLOC_CAP_SPIRAM);
        g->rx_sig = xSemaphoreCreateBinary();
        g->rx_lock = xSemaphoreCreateMutex();
        if (!g->rx || !g->rx_sig || !g->rx_lock) {
            ets_printf("gw: out of memory\n");
            return false;
        }
    }
    if (!driver_up()) {
        return false;
    }

    // Wait for the device list before choosing from it.
    //
    // new_dev_cb runs on the host stack's own task, and it was firing DURING
    // the open below rather than before it - so the list was still empty when
    // it was consulted, unit 0 opened "whichever" and never learned which one
    // it had taken. With one drive that only made the log say 255; with two it
    // would have sent unit 1 to open the device unit 0 was already using.
    for (int w = 0; w < 40 && s_nfound < unit + 1; w++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_nfound < unit + 1) {
        if (unit > 0) {
            return false;             // no second Greaseweazle; normal setup
        }
        ets_printf("gw: no Greaseweazle on the USB port (%04x:%04x)\n", GW_VID, GW_PID);
        return false;
    }

    cdc_acm_host_open_config_t cfg = {};
    cfg.vid = GW_VID;
    cfg.pid = GW_PID;
    cfg.interface_idx = 0;
    // Name the device. Every Greaseweazle shares a VID and PID, so asking for
    // that pair again would hand back the one another unit already holds; the
    // USB address is the only thing that tells them apart.
    cfg.dev_addr = 0;
    for (int i = 0; i < s_nfound; i++) {
        bool taken = false;
        for (int u = 0; u < GW_UNITS; u++) {
            if (s_gw[u].dev && s_gw[u].addr == s_found[i]) {
                taken = true;
                break;
            }
        }
        if (!taken) {
            cfg.dev_addr = s_found[i];
            break;
        }
    }
    if (!cfg.dev_addr) {
        return false;                     // every one found is already in use
    }
    cfg.connection_timeout_ms = 1000;
    cfg.out_buffer_size = 512;
    cfg.in_buffer_size = 4096;        // fewer, larger callbacks during a read
    cfg.event_cb = gw_event_cb;
    cfg.data_cb = rx_cb;
    cfg.user_arg = g;

    if (cdc_acm_host_open(&cfg, &g->dev) != ESP_OK) {
        if (unit == 0) {
            ets_printf("gw: no Greaseweazle on the USB port (%04x:%04x)\n", GW_VID, GW_PID);
        }
        g->dev = nullptr;
        return false;
    }
    g->addr = cfg.dev_addr;   // so the next unit does not open it again
    ets_printf("gw: unit %d opened, USB address %u\n", unit, g->addr);

    // The device ignores the line coding, but some hosts will not open the pipe
    // without one being set.
    cdc_acm_line_coding_t lc = {115200, 0, 0, 8};
    cdc_acm_host_line_coding_set(g->dev, &lc);

    const bool ok = gw_get_info();
    if (!ok) {
        ets_printf("gw: unit %d did not answer GetInfo\n", unit);
    }
    return ok;
}

// Called once at boot, purely to say what is plugged in.
extern "C" bool gw_probe(void) {
    return gw_probe_unit(0);
}
