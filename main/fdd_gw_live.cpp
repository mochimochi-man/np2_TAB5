#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <compiler.h>
#include <pccore.h>
#include <cpucore.h>
#include <io/iocore.h>
#include <diskimage/fddfile.h>

#include "gw_live.h"

extern "C" int ets_printf(const char *fmt, ...);

extern "C" _FDDFUNC fddfunc[MAX_FDDFILE];
extern "C" void np2lamp_fdd(REG8 drv);

#define GW_LIVE_MARK ":gw/live"

#define GW_DRIVES    2
#define GW_MAX_TRK   168
#define GW_MAX_SEC   32

#define GW_POOL      (2 * 1024 * 1024)

#define GW_REVS      3
#define GW_REVS_SLOW 5
#define GW_TRIES     4

struct live_trk {
    gw_sector_t *sec;
    int          nsec;
    bool         loaded;
};

struct live_drv {
    live_trk  trk[GW_MAX_TRK];
    uint8_t  *pool;
    size_t    pool_used;
    bool      running;
    bool      mounted;
    int       cyls, spt, secsize;
    uint32_t  rests;
};

static live_drv s_dr[GW_DRIVES];

static bool     s_machine_running;
static void door_opened(int drive);
static uint32_t s_seen_rest[GW_DRIVES];
static int      s_open_frames[GW_DRIVES];
static OEMCHAR  s_hidden[GW_DRIVES];

static uint8_t *s_scratch;

static inline live_drv *drv_of(int d) {
    return (d >= 0 && d < GW_DRIVES) ? &s_dr[d] : nullptr;
}

static int merge_capture(const gw_sector_t *cap, int ncap,
                         gw_sector_t *best, int nbest, uint8_t *pool_base,
                         size_t pool_cap, size_t *pool_used_out) {
    size_t used = *pool_used_out;
    for (int i = 0; i < ncap; i++) {
        const gw_sector_t *c = &cap[i];
        int slot = -1;
        for (int k = 0; k < nbest; k++) {
            if (best[k].c == c->c && best[k].h == c->h &&
                best[k].r == c->r && best[k].n == c->n) {
                slot = k;
                break;
            }
        }
        if (slot < 0) {
            if (nbest >= GW_MAX_SEC || used + c->len > pool_cap) {
                continue;
            }
            slot = nbest++;
            best[slot] = *c;
            best[slot].off = (uint32_t)used;
            used += c->len;
            memcpy(pool_base + best[slot].off, s_scratch + c->off, c->len);
            continue;
        }

        if (c->data_ok && !best[slot].data_ok) {
            memcpy(pool_base + best[slot].off, s_scratch + c->off, c->len);
            best[slot].data_ok = 1;
            best[slot].deleted = c->deleted;
            best[slot].no_data = 0;
            best[slot].dam_gap = c->dam_gap;
        }
    }
    *pool_used_out = used;
    return nbest;
}

static void flush_tracks(live_drv *d) {
    for (int i = 0; i < GW_MAX_TRK; i++) {
        free(d->trk[i].sec);
        d->trk[i].sec = nullptr;
        d->trk[i].nsec = 0;
        d->trk[i].loaded = false;
    }
    d->pool_used = 0;
}

static bool track_load_locked(int drive, int trk) {
    live_drv *d = drv_of(drive);
    if (!d || !d->pool || trk < 0 || trk >= GW_MAX_TRK) {
        return false;
    }

    const uint32_t rests = gw_live_rest_count(drive);
    if (d->mounted && rests != d->rests) {
        d->rests = rests;
        flush_tracks(d);
        ets_printf("gwfdd%d: drive rested - re-reading the disk that is in it now\n",
                   drive + 1);
    }
    if (d->trk[trk].loaded) {
        return d->trk[trk].nsec > 0;
    }
    const int cyl = trk >> 1;
    const int head = trk & 1;

    if (!s_scratch) {
        s_scratch = (uint8_t *)heap_caps_malloc((size_t)GW_MAX_SEC * GW_SECT_MAX_LEN,
                                                MALLOC_CAP_SPIRAM);
        if (!s_scratch) {
            return false;
        }
    }

    static gw_sector_t cap[GW_MAX_SEC];
    static gw_sector_t best[GW_MAX_SEC];
    int nbest = 0;
    uint8_t *base = d->pool + d->pool_used;
    const size_t cap_bytes = (d->pool_used < GW_POOL) ? (GW_POOL - d->pool_used) : 0;
    size_t used = 0;
    int attempts = 0;
    const int64_t t_start = esp_timer_get_time();

    for (int t = 0; t < GW_TRIES; t++) {
        const int n = gw_live_read_track(drive, cyl, head,
                                         (t == 0) ? GW_REVS : GW_REVS_SLOW,
                                         cap, GW_MAX_SEC, s_scratch,
                                         (size_t)GW_MAX_SEC * GW_SECT_MAX_LEN);
        attempts++;
        if (n <= 0) {
            continue;
        }
        nbest = merge_capture(cap, n, best, nbest, base, cap_bytes, &used);
        int bad = 0;
        for (int k = 0; k < nbest; k++) {
            if (!best[k].data_ok) {
                bad++;
            }
        }
        if (!bad) {
            break;
        }
    }

    if (nbest <= 0) {
        d->trk[trk].nsec = 0;
        __atomic_store_n(&d->trk[trk].loaded, true, __ATOMIC_RELEASE);
        ets_printf("gwfdd%d: C%d H%d unreadable\n", drive + 1, cyl, head);
        return false;
    }

    d->trk[trk].sec = (gw_sector_t *)heap_caps_malloc(sizeof(gw_sector_t) * nbest,
                                                      MALLOC_CAP_SPIRAM);
    if (!d->trk[trk].sec) {
        d->trk[trk].nsec = 0;
        __atomic_store_n(&d->trk[trk].loaded, true, __ATOMIC_RELEASE);
        return false;
    }
    for (int k = 0; k < nbest; k++) {
        best[k].off += (uint32_t)d->pool_used;
    }
    memcpy(d->trk[trk].sec, best, sizeof(gw_sector_t) * nbest);
    d->trk[trk].nsec = nbest;
    d->pool_used += used;
    __atomic_store_n(&d->trk[trk].loaded, true, __ATOMIC_RELEASE);

    int bad = 0, odd = 0;
    for (int i = 0; i < nbest; i++) {
        if (!d->trk[trk].sec[i].data_ok) bad++;
        if (d->trk[trk].sec[i].n != d->trk[trk].sec[0].n) odd++;
    }
    const unsigned ms = (unsigned)((esp_timer_get_time() - t_start) / 1000);
    if (bad || odd) {
        ets_printf("gwfdd%d: C%d H%d %d sectors, %d attempt%s, %ums%s%s\n",
                   drive + 1, cyl, head, nbest, attempts, attempts == 1 ? "" : "s", ms,
                   bad ? " - BAD CRC remains (kept as read)" : "",
                   odd ? " - mixed sector sizes" : "");

        for (int i = 0; i < nbest; i++) {
            const gw_sector_t *p = &d->trk[trk].sec[i];
            ets_printf("gwfdd%d:   C%u H%u R%u N%u %u bytes  data %s%s  @%uus dam+%u\n",
                       drive + 1, p->c, p->h, p->r, p->n, (unsigned)p->len,
                       p->no_data ? "NONE" : p->data_ok ? "ok" : "BAD",
                       p->deleted ? "  (deleted mark)" : "",
                       (unsigned)p->pos_us, (unsigned)p->dam_gap);
        }
    } else {
        ets_printf("gwfdd%d: C%d H%d %d sectors, %ums%s\n", drive + 1, cyl, head,
                   nbest, ms, attempts > 1 ? " (recovered on retry)" : "");
    }
    return true;
}

static bool track_load(int drive, int trk) {
    return track_load_locked(drive, trk);
}

static int s_misses;
static bool s_last_ma;

static bool want_mfm_now(void) {
    if (fdc.mf != 0xff) {
        return (fdc.mf & 0x40) != 0;
    }
    return (CPU_AH & 0x40) != 0;
}

static gw_sector_t *find_sector(int drive, int trk) {
    live_drv *d = drv_of(drive);
    s_last_ma = false;
    if (!d || !track_load(drive, trk)) {
        return nullptr;
    }

    const bool want_mfm = want_mfm_now();

    for (int i = 0; i < d->trk[trk].nsec; i++) {
        gw_sector_t *p = &d->trk[trk].sec[i];
        if (p->c == fdc.C && p->h == fdc.H && p->r == fdc.R && p->n == fdc.N) {
            if (want_mfm == (p->fm != 0)) {
                continue;
            }
            return p;
        }
    }

    bool any_same_density = false;
    for (int i = 0; i < d->trk[trk].nsec; i++) {
        if (want_mfm != (d->trk[trk].sec[i].fm != 0)) {
            any_same_density = true;
            break;
        }
    }
    s_last_ma = !any_same_density;
    if (!any_same_density) {
        fdc.stat[fdc.us] = (UINT32)(fdc.us | (fdc.hd << 2)) | FDCRLT_IC0 | FDCRLT_MA;
    }

    if (s_misses < 12) {
        s_misses++;
        ets_printf("gwfdd%d: MISS want C%u H%u R%u N%u %s on track %d -> %s (has %d:",
                   drive + 1, fdc.C, fdc.H, fdc.R, fdc.N,
                   want_mfm ? "MFM" : "FM", trk,
                   any_same_density ? "ND" : "MA", d->trk[trk].nsec);
        for (int i = 0; i < d->trk[trk].nsec && i < 12; i++) {
            ets_printf(" R%uN%u%s", d->trk[trk].sec[i].r, d->trk[trk].sec[i].n,
                       d->trk[trk].sec[i].fm ? "(FM)" : "");
        }
        ets_printf(")\n");
    }
    return nullptr;
}

static inline int cur_trk(void) {
    return (fdc.treg[fdc.us] << 1) + fdc.hd;
}

static uint64_t s_clk64;
static UINT32   s_clk_last;
static uint64_t s_busy_until;

static uint64_t now_clk(void) {
    const UINT32 c = (UINT32)(CPU_CLOCK + CPU_BASECLOCK - CPU_REMCLOCK);
    s_clk64 += (UINT32)(c - s_clk_last);
    s_clk_last = c;
    return s_clk64;
}

static inline uint64_t us_clk(uint32_t us) {
    return (uint64_t)us * pccore.realclock / 1000000u;
}

static inline uint32_t byte_us(const gw_sector_t *p) {
    const uint32_t b = (p->rev_us > 180000) ? 32 : 16;
    return p->fm ? b * 2 : b;
}

static uint64_t until_pos(uint64_t at, uint32_t pos_us, uint32_t rev_us) {
    const uint64_t rev = us_clk(rev_us);
    if (!rev) {
        return 0;
    }
    const uint64_t want = us_clk(pos_us) % rev;
    return (want + rev - at % rev) % rev;
}

static uint64_t cmd_start(void) {
    const uint64_t now = now_clk();
    return (s_busy_until > now) ? s_busy_until : now;
}

static void cmd_done_at(uint64_t end) {
    const uint64_t now = now_clk();
    s_busy_until = end;
    const uint64_t d = (end > now) ? end - now : 0;
    fdc_int_delay = (d < 512) ? 0 : (UINT32)((d > 0x7fff0000u) ? 0x7fff0000u : d);
}

static void cmd_not_found(const gw_sector_t *any) {
    if (any && any->rev_us) {
        const uint64_t at = cmd_start();
        cmd_done_at(at + until_pos(at, 0, any->rev_us) + us_clk(any->rev_us));
    }
}

static void cmd_sector(const gw_sector_t *p) {
    if (!p->rev_us) {
        return;
    }
    const uint64_t at = cmd_start();
    const uint32_t field = (7 + 22 + 12 + 4 + (uint32_t)p->len + 2) * byte_us(p);
    cmd_done_at(at + until_pos(at, p->pos_us, p->rev_us) + us_clk(field));
}

static void set_status(const gw_sector_t *p) {

    UINT8 st0 = (UINT8)((fdc.hd << 2) | fdc.us);
    UINT8 st1 = 0, st2 = 0;
    UINT8 result = 0x00;

    if (p->deleted) {
        st2 |= 0x40;
    }
    if (!p->data_ok) {
        st0 |= 0x40;
        st1 |= 0x20;
        st2 |= 0x20;
        result = 0xb0;
    }
    fdc.stat[fdc.us] = (UINT32)st0 | ((UINT32)st1 << 8) | ((UINT32)st2 << 16);
    fddlasterror = result;
}

static void forget(int drive) {
    live_drv *d = drv_of(drive);
    if (!d) {
        return;
    }
    if (d->running) {
        gw_live_end(drive);
        d->running = false;
    }
    for (int i = 0; i < GW_MAX_TRK; i++) {
        free(d->trk[i].sec);
        d->trk[i].sec = nullptr;
        d->trk[i].nsec = 0;
        d->trk[i].loaded = false;
    }
    heap_caps_free(d->pool);
    d->pool = nullptr;
    d->pool_used = 0;
    d->mounted = false;
    if (drive < MAX_FDDFILE &&
        !milstr_cmp((const OEMCHAR *)np2cfg.fddfile[drive], OEMTEXT(GW_LIVE_MARK))) {
        np2cfg.fddfile[drive][0] = '\0';
    }
}

static BRESULT gw_eject(FDDFILE fdd) {
    forget((int)(fdd - fddfile));
    return SUCCESS;
}

static BRESULT gw_diskaccess(FDDFILE fdd) {
    return (CTRL_FDMEDIA != fdd->inf.xdf.disktype) ? FAILURE : SUCCESS;
}

static BRESULT gw_seek(FDDFILE fdd) {
    if ((CTRL_FDMEDIA != fdd->inf.xdf.disktype) ||
        (fdc.rpm[fdc.us] != fdd->inf.xdf.rpm) ||
        (fdc.ncn >= (UINT)(fdd->inf.xdf.tracks >> 1))) {
        return FAILURE;
    }
    return SUCCESS;
}

static BRESULT gw_seeksector(FDDFILE fdd) {
    if ((CTRL_FDMEDIA != fdd->inf.xdf.disktype) ||
        (fdc.rpm[fdc.us] != fdd->inf.xdf.rpm)) {
        fddlasterror = 0xe0;
        return FAILURE;
    }
    const gw_sector_t *found = find_sector((int)(fdd - fddfile), cur_trk());
    if (!found) {
        fddlasterror = s_last_ma ? 0xe0 : 0xc0;
        return FAILURE;
    }
    return SUCCESS;
}

static BRESULT gw_read(FDDFILE fdd) {
    const int drive = (int)(fdd - fddfile);
    live_drv *d = drv_of(drive);
    fddlasterror = 0x00;
    const gw_sector_t *p = find_sector(drive, cur_trk());
    if (!p || !d) {
        if (d && d->trk[cur_trk()].nsec > 0) {
            cmd_not_found(&d->trk[cur_trk()].sec[0]);
        }
        fddlasterror = s_last_ma ? 0xe0 : 0xc0;
        return FAILURE;
    }
    cmd_sector(p);

    if (p->no_data) {
        fdc.stat[fdc.us] = (UINT32)((fdc.hd << 2) | fdc.us | 0x40) | (0x01u << 8) | (0x01u << 16);
        fddlasterror = 0xf0;
        return FAILURE;
    }
    UINT size = (fdc.N < 8) ? (128u << fdc.N) : (128u << 8);
    fdc.bufcnt = (int)size;
    ZeroMemory(fdc.buf, size);
    if (size > p->len) {
        size = p->len;
    }
    if (size) {
        CopyMemory(fdc.buf, d->pool + p->off, size);
    }
    set_status(p);
    return SUCCESS;
}

static BRESULT gw_readdiag(FDDFILE fdd) {
    const int drive = (int)(fdd - fddfile);
    live_drv *d = drv_of(drive);
    const int trk = cur_trk();
    if (!d || !track_load(drive, trk) || d->trk[trk].nsec <= 0) {
        fddlasterror = 0xe0;
        return FAILURE;
    }
    const bool want_mfm = (fdc.mf != 0xff) ? ((fdc.mf & 0x40) != 0) : true;
    const bool check_density = (fdc.mf != 0xff);

    if (want_mfm) {
        int bad = 0;
        const int got = gw_live_read_diag(drive, trk >> 1, trk & 1, fdc.N, fdc.buf,
                                          sizeof(fdc.buf), &bad);
        if (got > 0) {
            fdc.bufcnt = got;
            UINT8 st0 = (UINT8)((fdc.hd << 2) | fdc.us), st1 = 0, st2 = 0;
            fddlasterror = 0x00;
            if (bad) {
                st0 |= 0x40;
                st1 |= 0x20;
                st2 |= 0x20;
                fddlasterror = 0xb0;
            }
            fdc.stat[fdc.us] = (UINT32)st0 | ((UINT32)st1 << 8) | ((UINT32)st2 << 16);
            {

                const gw_sector_t *s0 = &d->trk[trk].sec[0];
                if (s0->rev_us) {
                    const uint64_t at = cmd_start();
                    const uint32_t span = (uint32_t)got * byte_us(s0) * 3 / 2;
                    cmd_done_at(at + until_pos(at, 0, s0->rev_us)
                                + us_clk(span < s0->rev_us ? span : s0->rev_us));
                }
            }
            return SUCCESS;
        }
    }

    size_t total = 0;
    int fields = 0;

    fdc.stat[fdc.us] = (UINT32)((fdc.hd << 2) | fdc.us);
    fddlasterror = 0x00;
    for (int i = 0; i < d->trk[trk].nsec; i++) {
        const gw_sector_t *p = &d->trk[trk].sec[i];
        if (check_density && (want_mfm != (p->fm == 0))) {
            continue;
        }
        if (total + p->len > sizeof(fdc.buf)) {
            break;
        }
        CopyMemory(fdc.buf + total, d->pool + p->off, p->len);
        total += p->len;
        fields++;
        fdc.C = p->c;
        fdc.H = p->h;
        fdc.R = p->r;
        fdc.N = p->n;
    }
    if (!fields) {
        fddlasterror = 0xc0;
        return FAILURE;
    }
    fdc.bufcnt = (int)total;
    return SUCCESS;
}

static BRESULT gw_write(FDDFILE fdd) {
    const int drive = (int)(fdd - fddfile);
    live_drv *d = drv_of(drive);
    gw_sector_t *p = find_sector(drive, cur_trk());
    if (!p || !d) {
        fddlasterror = s_last_ma ? 0xe0 : 0xc0;
        return FAILURE;
    }
    cmd_sector(p);
    UINT size = (fdc.N < 8) ? (128u << fdc.N) : (128u << 8);
    if (size > p->len) {
        size = p->len;
    }
    if (size) {
        CopyMemory(d->pool + p->off, fdc.buf, size);
    }
    p->data_ok = 1;
    p->deleted = ((fdc.cmd & 0x09) == 0x09) ? 1 : 0;
    fddlasterror = 0x00;
    fdc.stat[fdc.us] = (UINT32)((fdc.hd << 2) | fdc.us);
    return SUCCESS;
}

static BRESULT gw_readid(FDDFILE fdd) {
    const int drive = (int)(fdd - fddfile);
    live_drv *d = drv_of(drive);
    if ((!fdc.mf) || (fdc.rpm[fdc.us] != fdd->inf.xdf.rpm) ||
        (CTRL_FDMEDIA != fdd->inf.xdf.disktype)) {
        fddlasterror = 0xe0;
        return FAILURE;
    }
    const int trk = cur_trk();
    if (!d || !track_load(drive, trk) || d->trk[trk].nsec <= 0) {
        fddlasterror = 0xe0;
        return FAILURE;
    }
    if (fdc.crcn >= (UINT)d->trk[trk].nsec) {
        fdc.crcn = 0;
    }
    const gw_sector_t *p = &d->trk[trk].sec[fdc.crcn++];

    if (p->rev_us) {
        const bool want_mfm = want_mfm_now();
        const uint64_t at = cmd_start();
        const gw_sector_t *best = nullptr;
        uint64_t best_wait = 0;
        for (int i = 0; i < d->trk[trk].nsec; i++) {
            const gw_sector_t *q = &d->trk[trk].sec[i];
            if (!q->id_ok || (want_mfm == (q->fm != 0))) {
                continue;
            }
            const uint64_t w = until_pos(at, q->pos_us, q->rev_us);
            if (!best || w < best_wait) {
                best = q;
                best_wait = w;
            }
        }
        if (!best) {
            cmd_not_found(p);
            fddlasterror = 0xe0;
            return FAILURE;
        }
        p = best;
        cmd_done_at(at + best_wait + us_clk(7 * byte_us(p)));
    }
    fdc.C = p->c;
    fdc.H = p->h;
    fdc.R = p->r;
    fdc.N = p->n;
    fddlasterror = 0x00;
    return SUCCESS;
}

static BRESULT gw_refuse(FDDFILE fdd) {
    (void)fdd;
    fddlasterror = 0xb0;
    return FAILURE;
}

static BRESULT gw_refuse_fmt(FDDFILE fdd, const UINT8 *ID) {
    (void)fdd; (void)ID;
    fddlasterror = 0xb0;
    return FAILURE;
}

static BOOL gw_notformatting(FDDFILE fdd) {
    (void)fdd;
    return FALSE;
}

static void describe_disk(int drive) {
    live_drv *d = drv_of(drive);
    if (!d || d->trk[0].nsec <= 0) {
        return;
    }

    const gw_sector_t *boot = nullptr;
    for (int i = 0; i < d->trk[0].nsec; i++) {
        if (d->trk[0].sec[i].r == 1) {
            boot = &d->trk[0].sec[i];
            break;
        }
    }
    if (!boot) {
        ets_printf("gwfdd%d: no R1 on cylinder 0 - this disk has no IPL sector\n",
                   drive + 1);
        return;
    }
    const uint8_t *b = d->pool + boot->off;

    char oem[9];
    for (int i = 0; i < 8; i++) {
        oem[i] = (b[3 + i] >= 0x20 && b[3 + i] < 0x7f) ? (char)b[3 + i] : '.';
    }
    oem[8] = 0;
    const unsigned bps = (unsigned)(b[11] | (b[12] << 8));
    const unsigned spc = b[13];
    const unsigned rootent = (unsigned)(b[17] | (b[18] << 8));
    const unsigned total = (unsigned)(b[19] | (b[20] << 8));
    const unsigned media = b[21];
    const bool looks_fat = (b[0] == 0xeb || b[0] == 0xe9) &&
                           (bps == 256 || bps == 512 || bps == 1024) &&
                           spc && rootent && total;

    ets_printf("gwfdd%d: IPL C%u H%u R%u N%u, first bytes %02x %02x %02x, OEM \"%s\"\n",
               drive + 1, boot->c, boot->h, boot->r, boot->n, b[0], b[1], b[2], oem);
    if (looks_fat) {
        ets_printf("gwfdd%d: FAT12: %u bytes/sector, %u/cluster, %u root entries,"
                   " %u sectors, media %02x\n",
                   drive + 1, bps, spc, rootent, total, media);
    } else {
        ets_printf("gwfdd%d: not a DOS filesystem - a game loader or a raw IPL\n",
                   drive + 1);
    }

    for (int row = 0; row < 4; row++) {
        char txt[17];
        for (int i = 0; i < 16; i++) {
            const uint8_t v = b[row * 16 + i];
            txt[i] = (v >= 0x20 && v < 0x7f) ? (char)v : '.';
        }
        txt[16] = 0;
        ets_printf("gwfdd%d:   %02x: %02x %02x %02x %02x %02x %02x %02x %02x "
                   "%02x %02x %02x %02x %02x %02x %02x %02x  |%s|\n",
                   drive + 1, row * 16,
                   b[row*16+0], b[row*16+1], b[row*16+2], b[row*16+3],
                   b[row*16+4], b[row*16+5], b[row*16+6], b[row*16+7],
                   b[row*16+8], b[row*16+9], b[row*16+10], b[row*16+11],
                   b[row*16+12], b[row*16+13], b[row*16+14], b[row*16+15], txt);
    }
}

static bool mount_live(int drv) {
    live_drv *d = drv_of(drv);
    if (!d || drv >= MAX_FDDFILE) {
        return false;
    }
    fdd_eject((REG8)drv);
    forget(drv);

    d->pool = (uint8_t *)heap_caps_malloc(GW_POOL, MALLOC_CAP_SPIRAM);
    if (!d->pool) {
        ets_printf("gwfdd%d: no memory for the track cache\n", drv + 1);
        return false;
    }

    if (!gw_live_begin(drv)) {
        ets_printf("gwfdd%d: no Greaseweazle for this drive, or it would not start\n",
                   drv + 1);
        forget(drv);
        return false;
    }
    d->running = true;

    if (!track_load(drv, 0)) {
        ets_printf("gwfdd%d: cylinder 0 is unreadable - is there a disk in it?\n",
                   drv + 1);
        forget(drv);
        return false;
    }

    int spt = 0;
    const int ncode = d->trk[0].sec[0].n;
    for (int i = 0; i < d->trk[0].nsec; i++) {
        const gw_sector_t *p = &d->trk[0].sec[i];
        if (p->n == ncode && p->r > spt && p->r <= 32) {
            spt = p->r;
        }
    }
    const int cyls = (ncode == 3 && spt == 8) ? 77 : 80;

    FDDFILE fdd = fddfile + drv;
    FDDFUNC fn = fddfunc + drv;
    ZeroMemory(fdd, sizeof(_FDDFILE));
    fdd->type = DISKTYPE_BETA;
    fdd->ro = 1;

    fdd->protect = 0;
    fdd->inf.xdf.headersize = 0;
    fdd->inf.xdf.tracks = (UINT8)(cyls * 2);
    fdd->inf.xdf.sectors = (UINT8)spt;
    fdd->inf.xdf.n = (UINT8)ncode;
    fdd->inf.xdf.disktype = DISKTYPE_2HD;
    fdd->inf.xdf.rpm = 0;
    milstr_ncpy(fdd->fname, OEMTEXT("(GreaseWeazle live)"), NELEMENTS(fdd->fname));
    milstr_ncpy(np2cfg.fddfile[drv], OEMTEXT(GW_LIVE_MARK), NELEMENTS(np2cfg.fddfile[drv]));

    fn->eject       = gw_eject;
    fn->diskaccess  = gw_diskaccess;
    fn->seek        = gw_seek;
    fn->seeksector  = gw_seeksector;
    fn->readdiag    = gw_readdiag;
    fn->read        = gw_read;
    fn->write       = gw_write;
    fn->readid      = gw_readid;
    fn->writeid     = gw_refuse;
    fn->formatinit  = gw_refuse;
    fn->formating   = gw_refuse_fmt;
    fn->isformating = gw_notformatting;
    fn->fdcresult   = TRUE;

    d->mounted = true;
    d->rests = gw_live_rest_count(drv);
    s_seen_rest[drv] = d->rests;
    s_open_frames[drv] = 0;
    if (s_machine_running) {
        door_opened(drv);
    }
    d->cyls = cyls;
    d->spt = spt;
    d->secsize = 128 << ncode;
    ets_printf("gwfdd%d: live drive mounted on FDD%d - %d cyl x 2 head x %d sect x %d bytes\n",
               drv + 1, drv + 1, d->cyls, d->spt, d->secsize);
    describe_disk(drv);
    return true;
}

extern "C" bool fdd_gw_live_mount(int drv) {
    return mount_live(drv);
}

#define GW_DOOR_FRAMES 30

static void door_opened(int drive) {
    if (drive < 0 || drive >= GW_DRIVES || s_open_frames[drive] > 0) {
        return;
    }
    s_hidden[drive] = fddfile[drive].fname[0];
    fddfile[drive].fname[0] = 0;
    s_open_frames[drive] = GW_DOOR_FRAMES;

    fdc.stat[drive] = FDCRLT_AI | FDCRLT_NR | drive;
    fdc.us = (UINT8)drive;
    fdc_interrupt();
    ets_printf("gwfdd%d: telling the machine the disk was changed\n", drive + 1);
}

extern "C" void fdd_gw_live_menu_opened(void) {
    for (int d = 0; d < GW_DRIVES; d++) {
        live_drv *p = drv_of(d);

        if (!p || !p->mounted) {
            continue;
        }
        flush_tracks(p);
        p->rests = gw_live_rest_count(d);
        s_seen_rest[d] = p->rests;
        door_opened(d);
        ets_printf("gwfdd%d: menu opened - the disk in the drive will be read again\n",
                   d + 1);
    }
}

extern "C" void fdd_gw_live_tick(void) {
    if (fdc_waiting) {
        fdc_resume();
    }
    for (int d = 0; d < GW_DRIVES; d++) {
        live_drv *p = drv_of(d);

        if (!p || !p->mounted || s_open_frames[d] <= 0) {
            continue;
        }
        if (--s_open_frames[d] == 0) {
            fddfile[d].fname[0] = s_hidden[d];

            if ((!(fdc.chgreg & 4)) || (fdc.ctrlreg & 0x08)) {
                fdc.stat[d] = FDCRLT_AI | d;
                fdc.us = (UINT8)d;
                fdc_interrupt();
            }
            ets_printf("gwfdd%d: drive ready again\n", d + 1);
        }
    }
}

extern "C" void fdd_gw_live_started(void) {
    s_machine_running = true;
}

extern "C" bool fdd_gw_live_mounted(int drv) {
    const live_drv *d = drv_of(drv);
    return d && d->mounted;
}

extern "C" bool fdd_gw_live_info(int drv, int *cyls, int *spt, int *secsize) {
    const live_drv *d = drv_of(drv);
    if (!d || !d->mounted) {
        return false;
    }
    if (cyls) *cyls = d->cyls;
    if (spt) *spt = d->spt;
    if (secsize) *secsize = d->secsize;
    return true;
}

extern "C" bool fdd_gw_live_is_mark(const char *name) {
    return name && !milstr_cmp((const OEMCHAR *)name, OEMTEXT(GW_LIVE_MARK));
}

extern "C" int fdd_gw_live_available(void) {
    return gw_live_count();
}
