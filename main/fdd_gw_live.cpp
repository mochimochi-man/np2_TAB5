// The floppies in the real drives, served to the emulator as they are read.
//
// np2kai reaches a disk through a table of function pointers, one per FDC
// command (_FDDFUNC in diskimage/fddfile.h). Every existing backend fills that
// table from a file. This one fills it from a physical disk, a track at a time,
// through a Greaseweazle - one per drive, so an installer that asks for disk B
// in drive 2 can be answered.
//
// WHY, rather than dumping to a file first: a disk image can only describe a
// regular disk. .HDM is a bare array of sectors and .NFD a fixed table of them,
// so a track carrying sectors the format has no room for - an R=240 of 2048
// bytes, a data field whose CRC is wrong on purpose - loses exactly those
// sectors when it is written out. They are what copy protection is made of, so
// the image that results is the disk with its protection removed. Serving the
// disk live writes nothing, drops nothing, and needs the original in the drive
// to run, which is the same condition the real machine imposes.
//
// It works: a protected disk reads here as 19 sectors on cylinder 3 - eight
// ordinary ones and eleven of 2048 bytes whose data CRC never checks out - and
// the game accepts it and moves on.
//
// HOW IT PLUGS IN: fddfile[] and fddfunc[] have external linkage even though
// only fddfile[] is declared in the header, so a backend can be installed from
// outside np2kai without modifying it. Nothing in components/np2kai is touched.
//
// WHAT IT DOES NOT DO: weak bits (data that reads differently on each
// revolution) and timing-based checks are not reproduced - a cached track has
// one value per byte and no time axis. See gw_live.h.

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <compiler.h>
#include <pccore.h>
#include <io/iocore.h>
#include <diskimage/fddfile.h>

#include "gw_live.h"

extern "C" int ets_printf(const char *fmt, ...);

// Declared here rather than in a header: fddfile.h exposes fddfile[] but not
// fddfunc[], though both are file-scope with external linkage in fddfile.c.
extern "C" _FDDFUNC fddfunc[MAX_FDDFILE];

// The name kept in np2cfg.fddfile[] for a live drive. It is not a path and
// never can be - no file on the card starts with a colon - so every place that
// tests whether the file exists correctly decides it does not, and the two
// places that must not (the boot-time remount and the media check) look for
// this string instead. Same trick, and same reason, as ":builtin/" for the ROMs.
#define GW_LIVE_MARK ":gw/live"

// ---- per-drive state -------------------------------------------------------
#define GW_DRIVES    2            // FDD1 and FDD2, one Greaseweazle each
#define GW_MAX_TRK   168          // 84 cylinders, two heads
#define GW_MAX_SEC   32

// A 2HD disk is about 1.26MB of sector data. Two megabytes leaves room for the
// extra sectors a protected track carries (cylinder 3 above is 30KB rather than
// 8KB) without ever needing to evict anything - which matters, because evicting
// would mean re-reading, and a re-read costs half a second.
#define GW_POOL      (2 * 1024 * 1024)

// Two revolutions is enough when the disk is good; a track that comes back with
// a bad CRC is read again, because a marginal sector often reads clean on the
// third go. Four attempts is where that stops paying.
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
    uint32_t  rests;      // the drive's rest count when this cache was filled
};

static live_drv s_dr[GW_DRIVES];

// See fdd_gw_live_tick(): the door-open pulse that tells the guest the disk
// in a live drive may have been changed.
static bool     s_machine_running;   // set once the emulator is past start-up
static void door_opened(int drive);  // defined below, next to the pulse
static uint32_t s_seen_rest[GW_DRIVES];
static int      s_open_frames[GW_DRIVES];
static OEMCHAR  s_hidden[GW_DRIVES];

// One scratch track, shared. Captures land here first so that several of them
// can be compared before anything is committed to a cache; only one drive is
// ever being read at a time, because the emulator serialises floppy access
// through the FDC.
static uint8_t *s_scratch;

static inline live_drv *drv_of(int d) {
    return (d >= 0 && d < GW_DRIVES) ? &s_dr[d] : nullptr;
}

// ---- loading ---------------------------------------------------------------
// Merge one capture into what is already known about this track.
//
// A sector that failed its CRC is kept, but it is not the final word: read the
// track again and the marginal ones often come back clean. A sector that is bad
// on EVERY attempt is left bad, which is the right answer twice over - on a
// worn disk it is the truth, and on a protected one the bad CRC is deliberate
// and the guest is waiting to be told about it.
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
        // Already have it. Only a clean read displaces what is stored.
        if (c->data_ok && !best[slot].data_ok) {
            memcpy(pool_base + best[slot].off, s_scratch + c->off, c->len);
            best[slot].data_ok = 1;
            best[slot].deleted = c->deleted;
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

static bool track_load(int drive, int trk) {
    live_drv *d = drv_of(drive);
    if (!d || !d->pool || trk < 0 || trk >= GW_MAX_TRK) {
        return false;
    }
    // The drive stopped since this cache was filled, so the disk in it may
    // not be the disk it was read from. Start again rather than serve the
    // one that has been taken out.
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
            break;                   // everything readable is readable
        }
    }

    // A track that will not read is remembered as empty rather than retried on
    // every access: an unformatted track is a legitimate thing for a disk to
    // have, and the emulator has to be told "no data" promptly.
    d->trk[trk].loaded = true;
    if (nbest <= 0) {
        d->trk[trk].nsec = 0;
        ets_printf("gwfdd%d: C%d H%d unreadable\n", drive + 1, cyl, head);
        return false;
    }

    d->trk[trk].sec = (gw_sector_t *)heap_caps_malloc(sizeof(gw_sector_t) * nbest,
                                                      MALLOC_CAP_SPIRAM);
    if (!d->trk[trk].sec) {
        d->trk[trk].nsec = 0;
        return false;
    }
    for (int k = 0; k < nbest; k++) {
        best[k].off += (uint32_t)d->pool_used;     // pool-absolute
    }
    memcpy(d->trk[trk].sec, best, sizeof(gw_sector_t) * nbest);
    d->trk[trk].nsec = nbest;
    d->pool_used += used;

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
        // A track that is not plain is worth spelling out. These are the IDs as
        // they lie on the disk, and on a protected track they are the whole
        // story: an image format would have had to renumber or drop them, and
        // the CRC flag beside each is what the guest is really asking about.
        for (int i = 0; i < nbest; i++) {
            const gw_sector_t *p = &d->trk[trk].sec[i];
            ets_printf("gwfdd%d:   C%u H%u R%u N%u %u bytes  data %s%s\n",
                       drive + 1, p->c, p->h, p->r, p->n, (unsigned)p->len,
                       p->data_ok ? "ok" : "BAD",
                       p->deleted ? "  (deleted mark)" : "");
        }
    } else {
        ets_printf("gwfdd%d: C%d H%d %d sectors, %ums%s\n", drive + 1, cyl, head,
                   nbest, ms, attempts > 1 ? " (recovered on retry)" : "");
    }
    return true;
}

// The FDC compares all four ID bytes, and so does this: a protected track can
// carry a sector whose ID says it belongs somewhere else, and it is only found
// when the guest asks for it by that ID.
static int s_misses;

static gw_sector_t *find_sector(int drive, int trk) {
    live_drv *d = drv_of(drive);
    if (!d || !track_load(drive, trk)) {
        return nullptr;
    }
    // The controller compares the density too: an MFM read must not be given a
    // single-density sector, and the FM read a game uses to check which disk is
    // in the drive must not be answered with an ordinary one.
    const bool want_mfm = (fdc.mf != 0xff) ? ((fdc.mf & 0x40) != 0) : true;
    const bool check_density = (fdc.mf != 0xff);

    for (int i = 0; i < d->trk[trk].nsec; i++) {
        gw_sector_t *p = &d->trk[trk].sec[i];
        if (p->c == fdc.C && p->h == fdc.H && p->r == fdc.R && p->n == fdc.N) {
            if (check_density && (want_mfm == (p->fm != 0))) {
                continue;
            }
            return p;
        }
    }
    // How a real controller refuses matters. Told to read in one density, it
    // searches for address marks in that density; finding none at all on the
    // whole track is a missing address mark, and finding headers that simply do
    // not match is no data. A protection routine can read a sector it knows is
    // absent purely to see which of the two comes back, so the difference has
    // to be reported rather than flattened into one error.
    bool any_same_density = false;
    {
        const bool want_mfm2 = (fdc.mf != 0xff) ? ((fdc.mf & 0x40) != 0) : true;
        for (int i = 0; i < d->trk[trk].nsec; i++) {
            if (fdc.mf == 0xff || want_mfm2 != (d->trk[trk].sec[i].fm != 0)) {
                any_same_density = true;
                break;
            }
        }
    }
    if (!any_same_density) {
        fdc.stat[fdc.us] = (UINT32)(fdc.us | (fdc.hd << 2)) | FDCRLT_IC0 | FDCRLT_MA;
    }

    // Worth saying out loud, and worth saying only a few times. A guest that
    // asks for a sector this disk does not carry gets an error back and may
    // well go on to execute whatever it had already loaded - which is how a
    // machine ends up resetting itself in a loop. What it asked for, against
    // what is actually on the track, is the whole diagnosis.
    if (s_misses < 12) {
        s_misses++;
        ets_printf("gwfdd%d: MISS want C%u H%u R%u N%u %s on track %d -> %s (has %d:",
                   drive + 1, fdc.C, fdc.H, fdc.R, fdc.N,
                   (fdc.cmd & 0x40) ? "MFM" : "FM", trk,
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

// Turn what the disk said into what the FDC reports. A sector whose data CRC
// failed is not an error to be hidden: the guest asked for it and has to be
// told, because on a protected disk that answer is what it is checking for.
static void set_status(const gw_sector_t *p) {
    UINT8 st0 = (UINT8)(fdc.hd << 2);
    UINT8 st1 = 0, st2 = 0;
    UINT8 result = 0x00;

    if (p->deleted) {
        st2 |= 0x40;                 // CM - control mark, deleted data
    }
    if (!p->data_ok) {
        st0 |= 0x40;                 // IC - abnormal termination
        st1 |= 0x20;                 // DE - data error
        st2 |= 0x20;                 // DD - data error in the data field
        result = 0xa0;               // FDD BIOS: CRC error
    }
    fdc.stat[fdc.us] = (UINT32)st0 | ((UINT32)st1 << 8) | ((UINT32)st2 << 16);
    fddlasterror = result;
}

// ---- the backend -----------------------------------------------------------
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

// Deliberately NOT fdd_seeksector_common(): that rejects any R above the
// regular sectors-per-track, which is precisely the sector a protected disk
// hides its check in. The ID is looked up on the real track instead.
static BRESULT gw_seeksector(FDDFILE fdd) {
    if ((CTRL_FDMEDIA != fdd->inf.xdf.disktype) ||
        (fdc.rpm[fdc.us] != fdd->inf.xdf.rpm)) {
        fddlasterror = 0xe0;
        return FAILURE;
    }
    if (!find_sector((int)(fdd - fddfile), cur_trk())) {
        fddlasterror = 0xc0;         // no such sector on this track
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
        fddlasterror = 0xc0;
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

// READ DIAGNOSTIC (uPD765 READ A TRACK) begins at index and returns the data
// fields continuously, irrespective of the CHRN values in the command. That is
// observably different from repeated READ DATA and is used by protected disks.
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
    size_t total = 0;
    int fields = 0;

    fdc.stat[fdc.us] = (UINT32)(fdc.hd << 2);
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

// Writes never reach the disk. The original stays exactly as it is - which is
// the point of running from it - while the guest still sees its write take
// effect, so saved games work for as long as the disk is mounted.
static BRESULT gw_write(FDDFILE fdd) {
    const int drive = (int)(fdd - fddfile);
    live_drv *d = drv_of(drive);
    gw_sector_t *p = find_sector(drive, cur_trk());
    if (!p || !d) {
        fddlasterror = 0xc0;
        return FAILURE;
    }
    UINT size = (fdc.N < 8) ? (128u << fdc.N) : (128u << 8);
    if (size > p->len) {
        size = p->len;
    }
    if (size) {
        CopyMemory(d->pool + p->off, fdc.buf, size);
    }
    p->data_ok = 1;                  // what was just written reads back clean
    p->deleted = ((fdc.cmd & 0x09) == 0x09) ? 1 : 0;
    fddlasterror = 0x00;
    fdc.stat[fdc.us] = (UINT32)(fdc.hd << 2);
    return SUCCESS;
}

// Hands back the real ID fields in the order they lie on the track, which is
// how a guest discovers a sector it could not have guessed the number of.
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
    fdc.C = p->c;
    fdc.H = p->h;
    fdc.R = p->r;
    fdc.N = p->n;
    fddlasterror = 0x00;
    return SUCCESS;
}


static BRESULT gw_refuse(FDDFILE fdd) {
    (void)fdd;
    fddlasterror = 0xb0;             // write protected
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

// ---- what is on this disk --------------------------------------------------
// Printed once at mount. A disk that will not boot is a good deal easier to
// reason about when its first sector has been looked at rather than guessed at.
static void describe_disk(int drive) {
    live_drv *d = drv_of(drive);
    if (!d || d->trk[0].nsec <= 0) {
        return;
    }
    // The IPL is the sector the machine loads first: C0 H0 R1.
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

    // A PC-98 FAT12 disk starts with a jump and an OEM name, then the BIOS
    // parameter block. A game loader usually has neither.
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
    // The first 64 bytes, because a loader often signs itself there.
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

// ---- mounting --------------------------------------------------------------
// Reads cylinder 0 to find out what shape the disk is, then installs itself as
// the backend for that drive. Drive N is served by Greaseweazle unit N.
extern "C" bool fdd_gw_live_mount(int drv) {
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

    // The shape of the disk, taken from the track rather than assumed. Only the
    // regular sectors count towards it: the odd ones are the reason this exists
    // and must not move the geometry.
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
    fdd->protect = 1;
    fdd->inf.xdf.headersize = 0;
    fdd->inf.xdf.tracks = (UINT8)(cyls * 2);
    fdd->inf.xdf.sectors = (UINT8)spt;
    fdd->inf.xdf.n = (UINT8)ncode;
    fdd->inf.xdf.disktype = DISKTYPE_2HD;
    fdd->inf.xdf.rpm = 0;
    milstr_ncpy(fdd->fname, OEMTEXT("(GreaseWeazle live)"), NELEMENTS(fdd->fname));
    // Written where a filename would go, so that saving the settings persists
    // the choice and the next boot can put the physical drive back before the
    // machine starts - without which there is no way to boot from a real disk,
    // the only reset available restarting the whole board.
    milstr_ncpy(np2cfg.fddfile[drv], OEMTEXT(GW_LIVE_MARK),
                NELEMENTS(np2cfg.fddfile[drv]));

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
    ets_printf("gwfdd%d: mounted on FDD%d - %d cyl x 2 head x %d sect x %d bytes\n",
               drv + 1, drv + 1, d->cyls, d->spt, d->secsize);
    describe_disk(drv);
    return true;
}

// ---- telling the guest the disk was changed --------------------------------
// See the note above fdd_gw_live_tick(). Kept as a pulse rather than a level:
// the guest has to see the transition, and it has to see the drive come back.

#define GW_DOOR_FRAMES 30       // about a second at the emulator's frame rate

// Arm the pulse. Called when a disk has been mounted, which is the one moment
// we know for certain that the disk in the drive is not the disk that was in it
// before.
static void door_opened(int drive) {
    if (drive < 0 || drive >= GW_DRIVES || s_open_frames[drive] > 0) {
        return;
    }
    s_hidden[drive] = fddfile[drive].fname[0];
    fddfile[drive].fname[0] = 0;
    s_open_frames[drive] = GW_DOOR_FRAMES;
    ets_printf("gwfdd%d: telling the machine the disk was changed\n", drive + 1);
}

// Called once per frame from the emulator's own loop, which is where disk
// access happens too - so nothing here can land in the middle of a transfer.
extern "C" void fdd_gw_live_tick(void) {
    for (int d = 0; d < GW_DRIVES; d++) {
        live_drv *p = drv_of(d);

        if (!p || !p->mounted || s_open_frames[d] <= 0) {
            continue;
        }
        if (--s_open_frames[d] == 0) {
            fddfile[d].fname[0] = s_hidden[d];          // the door closes
            ets_printf("gwfdd%d: drive ready again\n", d + 1);
        }
    }
}

// The start-up mounts happen before the machine is running and must not look
// like a disk being swapped under it.
extern "C" void fdd_gw_live_started(void) {
    s_machine_running = true;
}

extern "C" bool fdd_gw_live_mounted(int drv) {
    const live_drv *d = drv_of(drv);
    return d && d->mounted;
}

// What was found on the disk, for the menu to show.
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

// Is this saved name the live drive rather than a file?
extern "C" bool fdd_gw_live_is_mark(const char *name) {
    return name && !milstr_cmp((const OEMCHAR *)name, OEMTEXT(GW_LIVE_MARK));
}

// How many Greaseweazles are plugged in, so the menu can offer a live drive
// only where there is hardware for it.
extern "C" int fdd_gw_live_available(void) {
    return gw_live_count();
}
