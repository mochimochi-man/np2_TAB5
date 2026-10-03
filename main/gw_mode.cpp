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
extern "C" bool gw_probe(void);
static bool gw_probe_unit(int unit);
static bool unit_select(int unit);
static const char *serial_of(uint8_t addr);
static uint8_t port_of(uint8_t addr);

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

#define GW_RX_CAP (1024 * 1024)

#define GW_UNITS 2

struct gw_dev {
    cdc_acm_dev_hdl_t dev;
    uint8_t          *rx;
    volatile size_t   rx_len;
    size_t            rx_pos;
    volatile bool     rx_ovf;
    SemaphoreHandle_t rx_sig;
    SemaphoreHandle_t rx_lock;
    uint32_t          freq;
    uint8_t           addr;
};

static gw_dev s_gw[GW_UNITS];

static gw_dev *g = &s_gw[0];

static bool rx_cb(const uint8_t *data, size_t len, void *arg) {

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
    return true;
}

static void rx_reset(void) {
    xSemaphoreTake(g->rx_lock, portMAX_DELAY);
    g->rx_len = 0;
    g->rx_pos = 0;
    g->rx_ovf = false;
    xSemaphoreGive(g->rx_lock);
    xSemaphoreTake(g->rx_sig, 0);
}

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

#define GW_RESYNC_MAX 8
#define GW_RESYNC_MS  200

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

static bool gw_cmd(const uint8_t *cmd, size_t len, int timeout_ms = 1000) {

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

static bool gw_motor(uint8_t unit, bool on) {
    const uint8_t b[4] = {CMD_MOTOR, 4, unit, (uint8_t)(on ? 1 : 0)};
    return gw_cmd(b, 4, 5000);
}

static const char *hw_model_name(uint8_t model) {
    switch (model) {
    case 1:  return "F1";
    case 4:  return "F7";
    case 7:  return "AT32F4";
    default: return "?";
    }
}

static bool gw_get_info(void) {
    const uint8_t cmd[3] = {CMD_GETINFO, 3, 0 };
    if (!gw_cmd(cmd, sizeof(cmd))) {
        return false;
    }
    uint8_t d[32];
    if (!rx_take(d, sizeof(d), 1000)) {
        ets_printf("gw: GetInfo returned no data\n");
        return false;
    }

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

    ets_printf("gw: sample_freq %u Hz (%u ps per tick)\n",
               (unsigned)g->freq,
               g->freq ? (unsigned)(1000000000000ULL / g->freq) : 0u);
    ets_printf("gw: mcu %u MHz, %u KB SRAM, %u KB USB buffer\n",
               mcu_mhz, mcu_sram_kb, usb_buf_kb);
    return g->freq != 0;
}

static bool gw_set_delays(void) {
    static const uint16_t d[5] = {
        10,
        10000,
        15,
        2000,
        30000,
    };
    uint8_t cmd[13] = {CMD_SETPARAMS, 13, 0 };
    for (int i = 0; i < 5; i++) {
        cmd[3 + i * 2] = (uint8_t)(d[i] & 0xff);
        cmd[4 + i * 2] = (uint8_t)(d[i] >> 8);
    }
    return gw_cmd(cmd, sizeof(cmd));
}

#define HIST_BUCKETS 64

struct flux_stats {
    uint32_t transitions;
    uint32_t index_ticks[8];
    int      nindex;
    uint32_t hist[HIST_BUCKETS];
};

typedef void (*flux_sink_t)(uint32_t ticks, void *arg);

template <typename Sink>
static bool decode_flux_core(const uint8_t *p, size_t n, flux_stats &st, Sink &&sink,
                             bool use_sink, bool cue_at_index, bool want_hist,
                             int stop_index) {
    const uint32_t bucket = g->freq / 4000000;
    memset(&st, 0, sizeof st);
    if (bucket == 0) {
        return false;
    }
    size_t i = 0;
    uint32_t ticks = 0;
    int64_t since_index = 0;
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
            if (op == 1) {
                if (st.nindex < 8) {
                    st.index_ticks[st.nindex] = (uint32_t)(since_index + ticks + v);
                }
                st.nindex++;
                if (stop_index && st.nindex >= stop_index) {
                    return true;
                }
                since_index = -(int64_t)(ticks + v);
                if (cue_at_index && !sink_cued) {
                    sink_cued = true;
                    first_after_index = true;
                }
            } else if (op == 2) {
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
            if (use_sink && sink_cued) {
                if (first_after_index) {
                    const int64_t clipped = since_index + ticks;
                    if (clipped > 0) {
                        sink((uint32_t)clipped);
                    }
                    first_after_index = false;
                } else {
                    sink(ticks);
                }
            }
            if (want_hist) {
                const uint32_t k = ticks / bucket;
                st.hist[(k < HIST_BUCKETS) ? k : (HIST_BUCKETS - 1)]++;
            }
            since_index += ticks;
            ticks = 0;
        }
    }
    return true;
}

static bool decode_flux(const uint8_t *p, size_t n, flux_stats &st,
                        flux_sink_t sink = nullptr, void *arg = nullptr,
                        bool cue_at_index = false) {
    return decode_flux_core(p, n, st, [&](uint32_t t) { sink(t, arg); },
                            sink != nullptr, cue_at_index, true, 0);
}

#define MAX_BITS (2048 * 1024)
#define MAX_SECT 32
#define SECT_STRIDE GW_SECT_MAX_LEN

static uint8_t *s_bits = nullptr;

#define PLL_PERIOD_ADJ 0.03f
#define PLL_PHASE_ADJ  0.55f

struct pll_state {
    float    cell;
    float    centre;
    float    phase;
    uint32_t nbits;
};

static void pll_sink(uint32_t flux, void *arg) {
    pll_state *st = (pll_state *)arg;
    st->phase += (float)flux;
    if (st->phase < st->cell / 2) {
        return;
    }

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

static bool pll_flux(const uint8_t *stream, size_t n, pll_state &pll, int stop_index) {
    flux_stats unused;
    return decode_flux_core(stream, n, unused, [&](uint32_t t) { pll_sink(t, &pll); },
                            true, true, false, stop_index);
}

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
            break;
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

static uint8_t mfm_byte(uint32_t *p, uint32_t nbits) {
    uint8_t b = 0;
    for (int k = 0; k < 8; k++) {
        if (*p + 2 > nbits) {
            return (uint8_t)(b << (8 - k));
        }
        (*p)++;
        b = (uint8_t)((b << 1) | s_bits[(*p)++]);
    }
    return b;
}

#define DAM_WINDOW 60

struct sect_hit {
    uint8_t  c, h, r, n;
    bool     id_ok;
    bool     data_ok;
    bool     deleted;
    bool     fm;
    bool     data_full;
    bool     has_data;
    uint8_t  dam_gap;
    int      seen;
    uint32_t pos_bits;
    uint8_t  cellmul;
};

static inline bool field_fits(uint32_t p, uint32_t len, uint32_t nbits) {
    return (uint64_t)p + (uint64_t)(len + 2) * 16 <= nbits;
}

static int sect_find(const sect_hit *v, int cnt, uint8_t c, uint8_t h,
                    uint8_t r, uint8_t n) {
    for (int k = 0; k < cnt; k++) {
        if (v[k].c == c && v[k].h == h && v[k].r == r && v[k].n == n) {
            return k;
        }
    }
    return -1;
}

static int mfm_scan(uint32_t nbits, sect_hit *out, int max_out,
                    uint8_t *data, size_t dcap) {
    static const uint8_t a1[3] = {0xa1, 0xa1, 0xa1};
    int cnt = 0;
    int pending = -1;
    uint32_t pending_end = 0;
    uint64_t sr = 0;

    for (uint32_t i = 0; i < nbits; i++) {
        sr = (sr << 1) | s_bits[i];
        if ((sr & 0xffffffffffffULL) != 0x448944894489ULL) {
            continue;
        }
        uint32_t p = i + 1;
        const uint8_t mark = mfm_byte(&p, nbits);

        if (mark == 0xfe) {
            uint8_t id[6];
            for (int k = 0; k < 6; k++) {
                id[k] = mfm_byte(&p, nbits);
            }
            uint16_t crc = crc16_ccitt(a1, 3, 0xffff);
            crc = crc16_ccitt(&mark, 1, crc);
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
                out[k].pos_bits = i;
                out[k].cellmul = 1;
            }
            out[k].id_ok = true;
            out[k].seen++;
            pending = k;
            pending_end = p;

        } else if (mark == 0xfb || mark == 0xf8) {
            if (pending < 0) {
                continue;
            }
            const int slot = pending;
            pending = -1;

            const uint32_t gap = (p - pending_end) / 16;
            if (gap > DAM_WINDOW) {
                if (!out[slot].has_data) {
                    out[slot].dam_gap = 255;
                }
                continue;
            }
            const bool first_data = !out[slot].has_data;
            out[slot].has_data = true;
            if (first_data) {
                out[slot].dam_gap = (uint8_t)gap;
            }
            const uint32_t len = 128u << (out[slot].n & 7);
            if (out[slot].data_ok) {
                continue;
            }

            const bool full = field_fits(p, len, nbits);
            if (!full && out[slot].data_full) {
                continue;
            }
            out[slot].data_full = full;

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

        }
    }
    return cnt;
}

struct track_out {
    sect_hit hit[MAX_SECT];
    int      nsect;
    uint8_t *data;
    float    cell;
    unsigned kbps;
    unsigned rpm10;
    uint32_t rev_us;
    float    us_per_bit;
};

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
                out[k].pos_bits = i;
                out[k].cellmul = 1;
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
            out[slot].has_data = true;
            if (out[slot].data_ok) {
                continue;
            }
            const uint32_t len = 128u << (out[slot].n & 7);
            uint32_t p = i + 1;
            const bool full = field_fits(p, len, nbits);
            if (!full && out[slot].data_full) {
                continue;
            }
            out[slot].data_full = full;
            uint8_t *dst = (data && len <= SECT_STRIDE &&
                            (size_t)(slot + 1) * SECT_STRIDE <= dcap)
                           ? data + (size_t)slot * SECT_STRIDE : nullptr;
            const uint8_t mark = (w == FM_AM_DATA) ? 0xfb : 0xf8;
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

static bool track_decode(const uint8_t *stream, size_t n, const flux_stats &st,
                         track_out &out, bool verbose, float cell_bias) {
    out.nsect = 0;
    out.cell = estimate_cell(st) * cell_bias;
    out.rev_us = (st.nindex > 1 && st.index_ticks[1])
                 ? (uint32_t)((uint64_t)st.index_ticks[1] * 1000000u / g->freq) : 0;
    out.us_per_bit = out.cell * 1000000.0f / (float)g->freq;
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
    if (!pll_flux(stream, n, pll, 0)) {
        return false;
    }
    if (out.data) {
        memset(out.data, 0, (size_t)MAX_SECT * SECT_STRIDE);
    }
    out.nsect = mfm_scan(pll.nbits, out.hit, MAX_SECT,
                         out.data, (size_t)MAX_SECT * SECT_STRIDE);

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

    if (out.nsect < MAX_SECT) {
        pll_state fmpll = {out.cell * 2.0f, out.cell * 2.0f, 0.0f, 0};

        if (pll_flux(stream, n, fmpll, 3)) {
            const int base = out.nsect;
            const int got = fm_scan(fmpll.nbits, out.hit + base, MAX_SECT - base,
                                    out.data ? out.data + (size_t)base * SECT_STRIDE : nullptr,
                                    (size_t)(MAX_SECT - base) * SECT_STRIDE);
            for (int k = 0; k < got; k++) {
                out.hit[base + k].cellmul = 2;
            }
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

static int export_sectors(const track_out &t, gw_sector_t *out, int max_out,
                          uint8_t *data, size_t data_cap) {
    int n = 0;
    uint32_t off = 0;
    for (int k = 0; k < t.nsect && n < max_out; k++) {
        const sect_hit &h = t.hit[k];
        const uint32_t len = 128u << (h.n & 7);
        if (len > SECT_STRIDE || off + len > data_cap) {
            continue;
        }
        memcpy(data + off, t.data + (size_t)k * SECT_STRIDE, len);
        out[n].c = h.c;
        out[n].h = h.h;
        out[n].r = h.r;
        out[n].n = h.n;
        out[n].id_ok = h.id_ok ? 1 : 0;
        out[n].data_ok = h.data_ok ? 1 : 0;
        out[n].deleted = h.deleted ? 1 : 0;
        out[n].fm = h.fm ? 1 : 0;
        out[n].no_data = h.has_data ? 0 : 1;
        out[n].dam_gap = h.has_data ? h.dam_gap : 255;
        out[n].seen = (uint8_t)((h.seen > 255) ? 255 : h.seen);
        out[n].len = (uint16_t)len;
        out[n].off = off;

        out[n].pos_us = (uint32_t)((float)h.pos_bits * t.us_per_bit * (h.cellmul ? h.cellmul : 1));
        if (t.rev_us) {
            out[n].pos_us %= t.rev_us;
        }
        out[n].rev_us = t.rev_us;
        off += len;
        n++;
    }
    return n;
}

static int diag_scan(uint32_t nbits, uint32_t rev_end, int ncode,
                     uint8_t *out, size_t cap, int *crc_bad) {
    static const uint8_t a1[3] = {0xa1, 0xa1, 0xa1};
    const uint32_t chunk = 128u << (ncode & 7);
    size_t got = 0;
    uint64_t sr = 0;
    *crc_bad = 0;
    for (uint32_t i = 0; i < nbits && i < rev_end && got < cap; i++) {
        sr = (sr << 1) | s_bits[i];
        if ((sr & 0xffffffffffffULL) != 0x448944894489ULL) {
            continue;
        }
        uint32_t p = i + 1;
        const uint8_t mark = mfm_byte(&p, nbits);
        if (mark != 0xfb && mark != 0xf8) {
            continue;
        }
        uint16_t crc = crc16_ccitt(a1, 3, 0xffff);
        crc = crc16_ccitt(&mark, 1, crc);
        for (uint32_t k = 0; k < chunk; k++) {
            const uint8_t b = mfm_byte(&p, nbits);
            crc = crc16_ccitt(&b, 1, crc);
            if (got < cap) {
                out[got++] = b;
            }
        }
        uint8_t tail[2];
        tail[0] = mfm_byte(&p, nbits);
        tail[1] = mfm_byte(&p, nbits);
        if (crc16_ccitt(tail, 2, crc) != 0) {
            (*crc_bad)++;
        }
        i = p - 1;
        sr = 0;
    }
    return (int)got;
}

static int diag_from_stream(const uint8_t *stream, size_t n, int ncode,
                            uint8_t *out, size_t cap, int *crc_bad) {
    flux_stats st;
    if (!decode_flux(stream, n, st)) {
        return -1;
    }
    const float cell = estimate_cell(st);
    if (cell < 4.0f) {
        return 0;
    }
    if (!s_bits) {
        s_bits = (uint8_t *)heap_caps_malloc(MAX_BITS, MALLOC_CAP_SPIRAM);
        if (!s_bits) {
            return -1;
        }
    }
    pll_state pll = {cell, cell, 0.0f, 0};
    if (!pll_flux(stream, n, pll, 0)) {
        return -1;
    }

    const uint32_t rev_end = (st.nindex > 1 && st.index_ticks[1])
                             ? (uint32_t)((float)st.index_ticks[1] / cell) : pll.nbits;
    return diag_scan(pll.nbits, rev_end, ncode, out, cap, crc_bad);
}

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

            ok = track_decode(p, n, st, out, verbose, cell_bias);
        }
    }

    gw_cmd2(CMD_GETFLUXSTATUS);
    return ok;
}

static track_out s_live[GW_UNITS];
static bool s_live_on[GW_UNITS];
static bool s_spun[GW_UNITS];
static int64_t s_used_us[GW_UNITS];
static uint32_t s_rests[GW_UNITS];

#define GW_IDLE_HOLD_US 10000000
#define GW_SPINUP_MS    2000

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

static bool gw_rearm(int unit) {
    if (!g->dev) {
        return false;
    }
    if (!gw_cmd3(CMD_SETBUSTYPE, BUS_IBMPC) || !gw_set_delays() ||
        !gw_cmd3(CMD_SELECT, 0) || !gw_motor(0, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(GW_SPINUP_MS));
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

extern "C" void gw_live_keepalive(void) {
    static int64_t due_us = 0;
    const int64_t now = esp_timer_get_time();

    if (now < due_us) {
        return;
    }
    due_us = now + 3000000;

    for (int u = 0; u < GW_UNITS; u++) {
        if (!s_live_on[u] || !s_spun[u] || !unit_select(u) || !g->dev) {
            continue;
        }
        if (now - s_used_us[u] < GW_IDLE_HOLD_US) {
            gw_motor(0, true);
        } else {
            gw_motor(0, false);
            gw_cmd2(CMD_DESELECT);
            s_spun[u] = false;
            s_rests[u]++;
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

    s_used_us[unit] = esp_timer_get_time();
    if (!gw_spin_up(unit)) {
        return -1;
    }
    if (!track_read(cyl, head, revs, live, false)) {

        if (!gw_rearm(unit) || !track_read(cyl, head, revs, live, false)) {
            return -1;
        }
    }
    return export_sectors(live, out, max_out, data, data_cap);
}

extern "C" int gw_live_read_diag(int unit, int cyl, int head, int ncode,
                                 uint8_t *out, size_t cap, int *crc_bad) {
    if (!unit_select(unit) || !g->dev) {
        return -1;
    }
    s_used_us[unit] = esp_timer_get_time();
    if (!gw_spin_up(unit)) {
        return -1;
    }
    if (!gw_cmd3(CMD_SEEK, (uint8_t)cyl) || !gw_cmd3(CMD_HEAD, (uint8_t)head)) {
        return -1;
    }
    const uint8_t rf[8] = {CMD_READFLUX, 8, 0, 0, 0, 0, 2, 0};
    if (!gw_cmd(rf, sizeof(rf))) {
        return -1;
    }
    size_t end = 0;
    int r = -1;
    if (!rx_wait_zero(&end, 5000)) {
        rx_drain(200, 3000);
    } else {
        const uint8_t *p = g->rx + g->rx_pos;
        const size_t n = end - g->rx_pos;
        g->rx_pos = end + 1;
        r = diag_from_stream(p, n, ncode, out, cap, crc_bad);
    }
    gw_cmd2(CMD_GETFLUXSTATUS);
    return r;
}

extern "C" const char *gw_live_serial(int unit) {
    if (!unit_select(unit) || !g->dev) {
        return "";
    }
    return serial_of(g->addr);
}

extern "C" uint32_t gw_live_rest_count(int unit) {
    if (unit < 0 || unit >= GW_UNITS) {
        return 0;
    }
    return s_rests[unit];
}

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

static uint8_t s_found[GW_UNITS];
static char    s_serial[GW_UNITS][20];
static uint8_t s_port[GW_UNITS];
static int s_nfound = 0;

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

static const char *serial_of(uint8_t addr) {
    for (int i = 0; i < s_nfound; i++) {
        if (s_found[i] == addr) {
            return s_serial[i][0] ? s_serial[i] : "?";
        }
    }
    return "?";
}

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
            return;
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

    return true;
}

static bool gw_probe_unit(int unit) {
    if (!unit_select(unit)) {
        return false;
    }
    if (g->dev) {
        return g->freq != 0;
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

    for (int w = 0; w < 40 && s_nfound < unit + 1; w++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_nfound < unit + 1) {
        if (unit > 0) {
            return false;
        }
        ets_printf("gw: no Greaseweazle on the USB port (%04x:%04x)\n", GW_VID, GW_PID);
        return false;
    }

    cdc_acm_host_open_config_t cfg = {};
    cfg.vid = GW_VID;
    cfg.pid = GW_PID;
    cfg.interface_idx = 0;

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
        return false;
    }
    cfg.connection_timeout_ms = 1000;
    cfg.out_buffer_size = 512;
    cfg.in_buffer_size = 4096;
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
    g->addr = cfg.dev_addr;
    ets_printf("gw: unit %d opened, USB address %u\n", unit, g->addr);

    cdc_acm_line_coding_t lc = {115200, 0, 0, 8};
    cdc_acm_host_line_coding_set(g->dev, &lc);

    const bool ok = gw_get_info();
    if (!ok) {
        ets_printf("gw: unit %d did not answer GetInfo\n", unit);
    }
    return ok;
}

extern "C" bool gw_probe(void) {
    return gw_probe_unit(0);
}
