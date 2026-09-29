// Disk Image Reader Mode: show the INSIDE of the mounted PC-98 hard disk image
// to the PC, rather than the SD card the image sits on.
//
// SD Card Reader Mode (usb_msc.cpp) hands the host the card itself, so the PC
// sees HDD.NHD as a 125MB file it can do nothing with. This mode goes one level
// down: it finds the DOS volume inside that .NHD and presents that as the USB
// disk, so Explorer opens it and the PC-98's own files are just files.
//
// Three layers have to be peeled back, and each one is a place the image can
// turn out not to be what we assumed:
//
//   1. The .NHD container. A 512-byte header (T98HDDIMAGE.R0) carrying the
//      geometry, then the raw disk. np2kai's own reader does exactly
//      "offset = headersize + lba * sectorsize", so that part is not guesswork.
//
//   2. The PC-98 partition table, at disk sector 1 - NOT an MBR. Sixteen
//      32-byte entries addressing their partitions in CHS, which only converts
//      to an LBA with the geometry from step 1. Windows cannot read this and
//      would offer to format the disk, which is why the whole image is never
//      presented raw.
//
//   3. The FAT volume inside the partition. Its boot sector is what actually
//      gets presented at LBA 0, as a "superfloppy" - a volume with no partition
//      table at all, which Windows mounts happily.
//
// Every step is checked rather than trusted: the partition start is only
// accepted if a FAT BPB is actually sitting there. If the CHS conversion were
// wrong the check fails and this says so, instead of handing the host garbage.
//
// PC-98 hard disk volumes are often formatted with 1024-byte logical sectors,
// which Windows will not mount; those are shown as 512-byte blocks the same
// way as the floppies below (nhd_translate).
//
// A floppy image in FDD1 or FDD2 can be chosen instead (the menu asks which
// drive, and leaves the answer in NVS). HDM, D88, NFD and FDI are read the
// same way DiskMount98 reads them: a table of where each sector sits in the
// file, looked up by cylinder/head/sector. Those are then shown to the host as
// 512-byte blocks whatever the disk's own sector size, with the BPB rewritten
// to count in 512-byte units on the way out and put back on the way in (see
// fd_patch below) - a 2HD disk has 1024-byte sectors, and the file on the card
// keeps them.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <strings.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "nvs.h"

extern "C" void usb_msc_log(const char *fmt, ...);
#define ets_printf usb_msc_log
extern "C" bool sd_mount(void);

#define NVS_NS "pc98"
#define NVS_KEY_DRIVE "usbimgdrv"   // 0 = FDD1, 1 = FDD2, 2 = HDD (menu_disk.cpp)

// The drives' NVS keys, as menu_disk.cpp's save_settings() writes them.
static const char *const k_drive_key[3] = { "fdd0", "fdd1", "hdd" };
static const char *const k_drive_name[3] = { "FDD1", "FDD2", "HDD" };

// What the emulator's own disk paths look like: "/HDD.NHD" is relative to the
// SD root, which is mounted at /sd.
#define SD_PREFIX "/sd"

static FILE *s_fp = nullptr;
static uint64_t s_vol_off = 0;    // byte offset of the volume inside the file
static uint32_t s_blk_size = 512; // the volume's logical sector size
static uint32_t s_blk_count = 0;  // volume size in those sectors
static char s_status[64] = "not opened";
static char s_name[80] = "";

extern "C" const char *usb_image_status(void) { return s_status; }
extern "C" const char *usb_image_name(void) { return s_name; }
extern "C" uint32_t usb_image_block_count(void) { return s_blk_count; }
extern "C" uint16_t usb_image_block_size(void) { return (uint16_t)s_blk_size; }
extern "C" bool usb_image_ready(void) { return s_fp != nullptr && s_blk_count > 0; }

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, v); wr16(p + 2, v >> 16); }

static bool read_at(uint64_t off, void *buf, size_t len);

// ---- floppy images -----------------------------------------------------
//
// Only used while opening: every sector the image holds, per track.
struct FdSec {
    uint8_t c, h, r, n;
    uint16_t size;
    uint32_t off;
};
#define FD_TRACKS 164
#define FD_MAXSEC 64
static FdSec *s_trk;               // FD_TRACKS * FD_MAXSEC
static uint8_t s_trk_n[FD_TRACKS];
static const uint8_t *s_raw;       // the whole file, while it is parsed
static uint32_t s_raw_len;

// What stays after opening.
static bool s_is_fd = false;
static uint32_t *s_fd_map;         // logical sector -> file offset
#define FD_MISSING 0xffffffffu
static uint32_t s_fd_bps;          // the disk's own sector size
static uint32_t s_fd_secs;         // logical sectors in the volume

// One byte the host sees differently from the file: the BPB counted in
// 512-byte sectors, and the first byte of each FAT made to match the BPB's
// media byte (the same fix NHD512 makes on the PC). Each is swapped only when
// the byte holds the value it is expected to, so a byte the host has
// deliberately changed goes through as written.
struct FdPatch {
    uint32_t block;                // 512-byte block
    uint16_t idx;
    uint8_t orig, virt;
};
static FdPatch s_patch[64];
static int s_npatch;

static void fd_add(int t, uint8_t c, uint8_t h, uint8_t r, uint8_t n, uint32_t off, uint32_t size) {
    if (t < 0 || t >= FD_TRACKS || s_trk_n[t] >= FD_MAXSEC) {
        return;
    }
    if (size == 0 || size > 8192 || (uint64_t)off + size > s_raw_len) {
        return;
    }
    FdSec *s = &s_trk[t * FD_MAXSEC + s_trk_n[t]++];
    s->c = c; s->h = h; s->r = r; s->n = n;
    s->size = (uint16_t)size;
    s->off = off;
}

static const FdSec *fd_find(int t, int r, uint32_t size) {
    if (t < 0 || t >= FD_TRACKS) {
        return nullptr;
    }
    for (int i = 0; i < s_trk_n[t]; i++) {
        const FdSec *s = &s_trk[t * FD_MAXSEC + i];
        if (s->r == r && (size == 0 || s->size == size)) {
            return s;
        }
    }
    return nullptr;
}

static void fd_load_nfd0(void) {
    uint32_t ptr = rd32(s_raw + 0x110);
    for (int i = 0; i < 163; i++) {
        for (int j = 0; j < 26; j++) {
            const uint32_t e = 0x120 + (i * 26 + j) * 16;
            if (e + 16 > s_raw_len || s_raw[e] == 0xff) {
                continue;
            }
            const uint32_t size = 128u << (s_raw[e + 3] & 7);
            fd_add(i, s_raw[e], s_raw[e + 1], s_raw[e + 2], s_raw[e + 3], ptr, size);
            ptr += size;
        }
    }
}

static bool fd_load_nfd1(void) {
    uint32_t ptr = rd32(s_raw + 0x110);
    for (int i = 0; i < 164; i++) {
        const uint32_t th = rd32(s_raw + 0x120 + i * 4);
        if (th == 0) {
            continue;
        }
        if ((uint64_t)th + 16 > s_raw_len) {
            return false;
        }
        const uint32_t nsec = rd16(s_raw + th);
        const uint32_t ndiag = rd16(s_raw + th + 2);
        uint32_t p = th + 16;
        for (uint32_t j = 0; j < nsec && p + 16 <= s_raw_len; j++, p += 16) {
            const uint32_t size = 128u << (s_raw[p + 3] & 7);
            fd_add(i, s_raw[p], s_raw[p + 1], s_raw[p + 2], s_raw[p + 3], ptr, size);
            ptr += size * (1 + s_raw[p + 10]);
        }
        for (uint32_t j = 0; j < ndiag && p + 16 <= s_raw_len; j++, p += 16) {
            ptr += rd32(s_raw + p + 10) * (1 + s_raw[p + 9]);
        }
    }
    return true;
}

static void fd_load_d88(void) {
    uint32_t end = rd32(s_raw + 0x1c);
    if (end < 0x2b0 || end > s_raw_len) {
        end = s_raw_len;
    }
    uint32_t table_end = 0x20 + 164 * 4;
    for (int i = 0; i < 164; i++) {
        const uint32_t o = rd32(s_raw + 0x20 + i * 4);
        if (o != 0 && o < table_end) {
            table_end = o;
        }
    }
    for (int i = 0; 0x20 + i * 4 < (int)table_end && i < 164; i++) {
        uint32_t p = rd32(s_raw + 0x20 + i * 4);
        if (p == 0 || p + 16 > end) {
            continue;
        }
        const uint32_t nsec = rd16(s_raw + p + 4);
        for (uint32_t j = 0; j < nsec && p + 16 <= end; j++) {
            const uint32_t size = rd16(s_raw + p + 0x0e);
            fd_add(i, s_raw[p], s_raw[p + 1], s_raw[p + 2], s_raw[p + 3], p + 16, size);
            p += 16 + size;
        }
    }
}

// A plain dump, cylinder by cylinder, head by head, from `base`.
static void fd_load_flat(uint32_t base, int cyls, int heads, int spt, uint32_t bps) {
    const uint8_t n = (bps == 1024) ? 3 : (bps == 512) ? 2 : (bps == 256) ? 1 : 0;
    uint32_t off = base;
    for (int t = 0; t < cyls * heads; t++) {
        for (int s = 1; s <= spt; s++, off += bps) {
            const int track = (heads == 2) ? t : t * 2;
            fd_add(track, (uint8_t)(t / heads), (uint8_t)(t % heads), (uint8_t)s, n, off, bps);
        }
    }
}

// HDM and the other raw dumps carry no header, so the file size is the
// geometry. FDI has a 4096-byte header that states it.
static bool fd_load_raw(const char **fmt) {
    if (s_raw_len > 32) {
        const uint32_t hs = rd32(s_raw + 8), size = rd32(s_raw + 12), ss = rd32(s_raw + 16);
        const uint32_t spt = rd32(s_raw + 20), heads = rd32(s_raw + 24), cyls = rd32(s_raw + 28);
        if (hs >= 32 && (uint64_t)hs + size == s_raw_len && (ss == 256 || ss == 512 || ss == 1024) &&
            spt >= 1 && spt <= 26 && (heads == 1 || heads == 2) && cyls >= 1 && cyls <= 82 &&
            spt * heads * cyls * ss == size) {
            *fmt = "FDI";
            fd_load_flat(hs, (int)cyls, (int)heads, (int)spt, ss);
            return true;
        }
    }
    static const struct { uint32_t size; int cyls, spt; uint32_t bps; } geo[] = {
        { 1261568, 77,  8, 1024 },   // 2HD 1.25MB (HDM)
        { 1474560, 80, 18,  512 },   // 2HD 1.44MB
        { 1228800, 80, 15,  512 },   // 2HC 1.2MB
        {  737280, 80,  9,  512 },   // 2DD 720KB
        {  655360, 80,  8,  512 },   // 2DD 640KB
    };
    for (size_t i = 0; i < sizeof(geo) / sizeof(geo[0]); i++) {
        if (s_raw_len == geo[i].size) {
            *fmt = (geo[i].size == 1261568) ? "HDM" : "raw";
            fd_load_flat(0, geo[i].cyls, 2, geo[i].spt, geo[i].bps);
            return true;
        }
    }
    return false;
}

struct FdBpb {
    uint32_t bps, spc, res, nfats, root, total, media, fsz, spt, heads;
    bool guessed;
};

static bool fd_bpb_from(const uint8_t *b, uint32_t secsize, FdBpb *v) {
    v->bps = rd16(b + 11); v->spc = b[13]; v->res = rd16(b + 14); v->nfats = b[16];
    v->root = rd16(b + 17); v->total = rd16(b + 19); v->media = b[21];
    v->fsz = rd16(b + 22); v->spt = rd16(b + 24); v->heads = rd16(b + 26);
    if (v->total == 0) {
        v->total = rd32(b + 32);
    }
    v->guessed = false;
    if (v->bps != secsize || (v->bps != 256 && v->bps != 512 && v->bps != 1024)) return false;
    if (v->spc == 0 || (v->spc & (v->spc - 1)) != 0 || v->res < 1 || v->nfats < 1 || v->nfats > 2) return false;
    if (v->root == 0 || v->total == 0 || v->fsz == 0 || v->media < 0xf0) return false;
    if (v->spt < 1 || v->spt > 26 || v->heads < 1 || v->heads > 2) return false;
    return true;
}

// Older disks have no BPB; their layout is fixed by the track format.
static bool fd_bpb_guess(uint32_t secsize, FdBpb *v) {
    int spt = 0;
    for (int i = 0; i < s_trk_n[2]; i++) {
        const FdSec *s = &s_trk[2 * FD_MAXSEC + i];
        if (s->size == secsize && s->r > spt) spt = s->r;
    }
    static const uint32_t t[][6] = {
        // bps, spt, spc, root, total, fatsz
        { 1024,  8, 1, 192, 1232, 2 },
        {  512, 18, 1, 224, 2880, 9 },
        {  512, 15, 1, 224, 2400, 7 },
        {  512,  9, 2, 112, 1440, 3 },
        {  512,  8, 2, 112, 1280, 2 },
    };
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        if (t[i][0] != secsize || (int)t[i][1] != spt) continue;
        const FdSec *f = fd_find(0, 2, secsize);
        if (!f || s_raw[f->off] < 0xf0 || s_raw[f->off + 1] != 0xff) return false;
        v->bps = t[i][0]; v->spt = t[i][1]; v->spc = t[i][2]; v->root = t[i][3];
        v->total = t[i][4]; v->fsz = t[i][5];
        v->res = 1; v->nfats = 2; v->heads = 2;
        v->media = s_raw[f->off];
        v->guessed = true;
        return true;
    }
    return false;
}

static void fd_patch(uint32_t voff, uint8_t orig, uint8_t virt) {
    if (orig == virt || s_npatch >= (int)(sizeof(s_patch) / sizeof(s_patch[0]))) {
        return;
    }
    FdPatch *p = &s_patch[s_npatch++];
    p->block = voff / 512;
    p->idx = (uint16_t)(voff % 512);
    p->orig = orig;
    p->virt = virt;
}

// What the host is shown of the boot sector `o` (its first 512 bytes) and of
// the first byte of each FAT (`fat0`, as the file has them): the BPB counted in
// 512-byte sectors, and the FATs' media byte matching the BPB's. Fails if the
// layout does not fit a BPB counted that way.
static bool vol_translate(const uint8_t *o, const FdBpb *v, const uint8_t *fat0) {
    const uint32_t k = v->bps / 512;
    if (v->spc * k > 128 || v->res * k > 0xffff || v->fsz * k > 0xffff || v->nfats > 4) {
        return false;
    }
    uint8_t w[512];
    memcpy(w, o, 512);
    if (v->guessed) {
        w[0] = 0xeb; w[1] = 0x3c; w[2] = 0x90;
        w[16] = (uint8_t)v->nfats; wr16(w + 17, v->root); w[21] = (uint8_t)v->media;
        wr16(w + 26, v->heads);
    }
    wr16(w + 11, 512);
    w[13] = (uint8_t)(v->spc * k);
    wr16(w + 14, v->res * k);
    wr16(w + 22, v->fsz * k);
    wr16(w + 24, v->spt * k);
    wr32(w + 28, 0);   // shown as a volume on its own, from block 0
    if (v->total * k <= 0xffff) {
        wr16(w + 19, v->total * k);
        wr32(w + 32, 0);
    } else {
        wr16(w + 19, 0);
        wr32(w + 32, v->total * k);
    }
    w[510] = 0x55; w[511] = 0xaa;
    s_npatch = 0;
    for (int i = 0; i < 512; i++) {
        fd_patch(i, o[i], w[i]);
    }
    for (uint32_t f = 0; f < v->nfats; f++) {
        fd_patch((v->res + f * v->fsz) * v->bps, fat0[f], (uint8_t)v->media);
    }
    return true;
}

static bool open_fd(const char *path) {
    struct stat st;
    char full[96];
    snprintf(full, sizeof(full), "%s%s", SD_PREFIX, path);
    if (stat(full, &st) != 0 || st.st_size < 0x2b0 || st.st_size > 4 * 1024 * 1024) {
        snprintf(s_status, sizeof(s_status), "not a floppy image (size)");
        return false;
    }
    s_raw_len = (uint32_t)st.st_size;
    uint8_t *raw = (uint8_t *)heap_caps_malloc(s_raw_len, MALLOC_CAP_SPIRAM);
    s_trk = (FdSec *)heap_caps_calloc(FD_TRACKS * FD_MAXSEC, sizeof(FdSec), MALLOC_CAP_SPIRAM);
    memset(s_trk_n, 0, sizeof(s_trk_n));
    bool ok = false;
    const char *fmt = "?";
    FdBpb v = {};
    const FdSec *boot = nullptr;

    if (!raw || !s_trk) {
        snprintf(s_status, sizeof(s_status), "out of memory");
        goto done;
    }
    if (!read_at(0, raw, s_raw_len)) {
        snprintf(s_status, sizeof(s_status), "cannot read the image");
        goto done;
    }
    s_raw = raw;

    {
        const char *dot = strrchr(path, '.');
        const bool d88_ext = dot && (!strcasecmp(dot, ".d88") || !strcasecmp(dot, ".d98") ||
                                     !strcasecmp(dot, ".88d"));
        if (!memcmp(raw, "T98FDDIMAGE.R0", 14)) {
            fmt = "NFD r0";
            fd_load_nfd0();
        } else if (!memcmp(raw, "T98FDDIMAGE.R1", 14)) {
            fmt = "NFD r1";
            if (!fd_load_nfd1()) {
                snprintf(s_status, sizeof(s_status), "NFD track table is corrupted");
                goto done;
            }
        } else if (d88_ext || rd32(raw + 0x1c) == s_raw_len) {
            fmt = "D88";
            fd_load_d88();
        } else if (!fd_load_raw(&fmt)) {
            snprintf(s_status, sizeof(s_status), "unknown floppy image format");
            goto done;
        }
    }

    boot = fd_find(0, 1, 0);
    if (!boot) {
        snprintf(s_status, sizeof(s_status), "%s: no boot sector", fmt);
        goto done;
    }
    if (!fd_bpb_from(raw + boot->off, boot->size, &v) && !fd_bpb_guess(boot->size, &v)) {
        snprintf(s_status, sizeof(s_status), "%s: not a DOS disk (no BPB)", fmt);
        goto done;
    }

    // Logical sector -> where it is in the file.
    s_fd_map = (uint32_t *)heap_caps_malloc(v.total * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (!s_fd_map) {
        snprintf(s_status, sizeof(s_status), "out of memory");
        goto done;
    }
    for (uint32_t lba = 0; lba < v.total; lba++) {
        const uint32_t t = lba / v.spt;
        const int track = (int)((t / v.heads) * 2 + t % v.heads);
        const FdSec *s = fd_find(track, (int)(lba % v.spt) + 1, v.bps);
        s_fd_map[lba] = s ? s->off : FD_MISSING;
    }
    {
        // The FATs and the root directory must all be there; a file's data
        // may sit on a track the disk never had formatted.
        const uint32_t sys = v.res + v.nfats * v.fsz + (v.root * 32 + v.bps - 1) / v.bps;
        if (sys >= v.total) {
            snprintf(s_status, sizeof(s_status), "%s: invalid FAT layout", fmt);
            goto done;
        }
        for (uint32_t i = 0; i < sys; i++) {
            if (s_fd_map[i] == FD_MISSING) {
                snprintf(s_status, sizeof(s_status), "%s: FAT/directory sectors missing", fmt);
                goto done;
            }
        }
    }

    if (v.bps < 512) {
        snprintf(s_status, sizeof(s_status), "%s: %u-byte sectors not supported", fmt,
                 (unsigned)v.bps);
        goto done;
    }
    {
        uint8_t fat0[4];
        for (uint32_t f = 0; f < v.nfats; f++) {
            const uint32_t voff = (v.res + f * v.fsz) * v.bps;
            fat0[f] = raw[s_fd_map[voff / v.bps] + voff % v.bps];
        }
        if (!vol_translate(raw + boot->off, &v, fat0)) {
            snprintf(s_status, sizeof(s_status), "%s: layout cannot be shown in 512-byte sectors", fmt);
            goto done;
        }
    }

    s_is_fd = true;
    s_fd_bps = v.bps;
    s_fd_secs = v.total;
    s_vol_off = 0;
    s_blk_size = 512;
    // One block more than the disk has. A device exactly the size of a
    // standard floppy (1.44MB, 720KB, ...) is handed by Windows to its USB
    // floppy driver, which then asks for the flexible disk mode page (05h),
    // gets nothing it can use, and never mounts the volume. Any other size is
    // an ordinary disk. The extra block reads as zeros and refuses writes; the
    // FAT only ever goes as far as the BPB says.
    s_blk_count = v.total * (v.bps / 512) + 1;
    snprintf(s_status, sizeof(s_status), "%s, %u-byte sectors%s%s", fmt, (unsigned)v.bps,
             v.bps != 512 ? " shown as 512" : "", v.guessed ? ", no BPB" : "");
    ets_printf("usb_image: %s: %u sectors of %u bytes, %d bytes adjusted\n", fmt,
               (unsigned)v.total, (unsigned)v.bps, s_npatch);
    ok = true;

done:
    free(raw);
    free(s_trk);
    s_raw = nullptr;
    s_trk = nullptr;
    if (!ok) {
        free(s_fd_map);
        s_fd_map = nullptr;
    }
    return ok;
}

// Where 512-byte block `b` is in the file, or FD_MISSING. A floppy image goes
// through its sector table; a hard disk volume (s_fd_map == nullptr) is one
// contiguous run from s_vol_off.
#define BLK_MISSING UINT64_MAX
static uint64_t fd_block_off(uint32_t b) {
    const uint64_t voff = (uint64_t)b * 512;
    const uint32_t sec = (uint32_t)(voff / s_fd_bps);
    if (sec >= s_fd_secs) {
        return BLK_MISSING;
    }
    if (!s_fd_map) {
        return s_vol_off + voff;
    }
    if (s_fd_map[sec] == FD_MISSING) {
        return BLK_MISSING;
    }
    return s_fd_map[sec] + (uint32_t)(voff % s_fd_bps);
}

static bool fd_patched(uint32_t lba, uint32_t bytes) {
    for (int i = 0; i < s_npatch; i++) {
        if (s_patch[i].block >= lba && (uint64_t)(s_patch[i].block - lba) * 512 < bytes) {
            return true;
        }
    }
    return false;
}

static int32_t fd_read(uint32_t lba, uint8_t *buf, uint32_t bytes) {
    // A hard disk volume is read in one go, as it was before 512-byte blocks.
    const bool whole = !s_fd_map && fd_block_off(lba + (bytes - 1) / 512) != BLK_MISSING;
    if (whole && !read_at(s_vol_off + (uint64_t)lba * 512, buf, bytes)) {
        return -1;
    }
    for (uint32_t done = 0; done < bytes; done += 512) {
        const uint32_t b = lba + done / 512;
        const uint32_t n = (bytes - done < 512) ? bytes - done : 512;
        if (!whole) {
            const uint64_t off = fd_block_off(b);
            if (off == BLK_MISSING) {
                memset(buf + done, 0, n);    // a track the disk never had formatted
            } else if (!read_at(off, buf + done, n)) {
                return -1;
            }
        }
        for (int i = 0; i < s_npatch; i++) {
            const FdPatch *p = &s_patch[i];
            if (p->block == b && p->idx < n && buf[done + p->idx] == p->orig) {
                buf[done + p->idx] = p->virt;
            }
        }
    }
    return (int32_t)bytes;
}

static int32_t fd_write(uint32_t lba, const uint8_t *buf, uint32_t bytes) {
    static uint8_t tmp[512];
    if (!s_fd_map && !fd_patched(lba, bytes) && fd_block_off(lba + (bytes - 1) / 512) != BLK_MISSING) {
        const uint64_t off = s_vol_off + (uint64_t)lba * 512;
        if (fseek(s_fp, (long)off, SEEK_SET) != 0 || fwrite(buf, 1, bytes, s_fp) != bytes) {
            return -1;
        }
        return (int32_t)bytes;
    }
    for (uint32_t done = 0; done < bytes; done += 512) {
        const uint32_t b = lba + done / 512;
        const uint32_t n = (bytes - done < 512) ? bytes - done : 512;
        const uint64_t off = fd_block_off(b);
        if (off == BLK_MISSING) {
            return -1;
        }
        memcpy(tmp, buf + done, n);
        for (int i = 0; i < s_npatch; i++) {
            const FdPatch *p = &s_patch[i];
            if (p->block == b && p->idx < n && tmp[p->idx] == p->virt) {
                tmp[p->idx] = p->orig;
            }
        }
        if (fseek(s_fp, (long)off, SEEK_SET) != 0 || fwrite(tmp, 1, n, s_fp) != n) {
            return -1;
        }
    }
    return (int32_t)bytes;
}

static bool read_at(uint64_t off, void *buf, size_t len) {
    if (fseek(s_fp, (long)off, SEEK_SET) != 0) {
        return false;
    }
    return fread(buf, 1, len, s_fp) == len;
}

// Is this a FAT boot sector? Checked rather than assumed, because it is what
// tells us the partition table was read correctly.
static bool bpb_looks_sane(const uint8_t *sec, uint32_t *out_bps, uint32_t *out_total) {
    // NOT tested here: the 0x55AA at the end of the sector. That is a PC/AT
    // convention; a PC-98 boots through its own partition table and MS-DOS 3.30
    // for the PC-98 leaves those two bytes alone. Requiring them rejected a
    // perfectly good DOS volume, so the media descriptor below and the first
    // FAT entry (checked by the caller) do the identifying instead.
    const uint32_t bps = rd16(sec + 11);
    if (bps != 512 && bps != 1024 && bps != 2048 && bps != 4096) {
        return false;
    }
    const uint8_t spc = sec[13];
    if (spc == 0 || (spc & (spc - 1)) != 0 || spc > 128) {
        return false;
    }
    if (rd16(sec + 14) == 0) {   // reserved sectors: never zero on FAT
        return false;
    }
    if (sec[16] == 0 || sec[16] > 4) {   // number of FATs
        return false;
    }
    if (sec[21] < 0xf0) {                // media descriptor: 0xF0..0xFF
        return false;
    }
    uint32_t total = rd16(sec + 19);
    if (total == 0) {
        total = rd32(sec + 32);
    }
    if (total == 0) {
        return false;
    }
    *out_bps = bps;
    *out_total = total;
    return true;
}

static void dump(const char *what, const uint8_t *p, int n) {
    ets_printf("usb_image: %s\n", what);
    for (int i = 0; i < n; i += 16) {
        ets_printf("  %03x:", i);
        for (int j = 0; j < 16; j++) {
            ets_printf(" %02x", p[i + j]);
        }
        ets_printf("\n");
    }
}

// Try one candidate volume start (byte offset in the file). Fills the globals
// on success.
static bool try_volume(uint64_t off, const char *what) {
    uint8_t sec[512];
    if (!read_at(off, sec, sizeof(sec))) {
        return false;
    }
    uint32_t bps = 0, total = 0;
    if (!bpb_looks_sane(sec, &bps, &total)) {
        return false;
    }
    // The first FAT entry repeats the media descriptor, followed by 0xFF 0xFF.
    // A run of bytes that merely looks like a BPB will not also have that, so
    // this is the check that keeps the cylinder scan below honest.
    {
        const uint32_t reserved = rd16(sec + 14);
        uint8_t fat[512];
        if (!read_at(off + (uint64_t)reserved * bps, fat, sizeof(fat))) {
            return false;
        }
        if (fat[0] < 0xf0 || fat[1] != 0xff || fat[2] != 0xff) {
            // Worth saying out loud: the BPB was convincing, so this is either
            // a volume laid out in a way not accounted for here, or the wrong
            // offset - and either way the next person needs the bytes.
            ets_printf("usb_image: %s: BPB looks like FAT (%u-byte sectors, "
                       "%u reserved) but the first FAT entry is %02x %02x %02x\n",
                       what, (unsigned)bps, (unsigned)reserved,
                       fat[0], fat[1], fat[2]);
            return false;
        }
    }
    s_vol_off = off;
    s_blk_size = bps;
    s_blk_count = total;
    ets_printf("usb_image: %s -> FAT volume, %u sectors of %u bytes (%u MB)\n",
               what, (unsigned)total, (unsigned)bps,
               (unsigned)(((uint64_t)total * bps) >> 20));
    return true;
}

// A hard disk volume found by try_volume() is shown the same way as a floppy
// when it needs to be: PC-98 DOS formats hard disks with 1024-byte logical
// sectors, which Windows will not mount, and EPSON's DOS leaves the FATs' media
// byte different from the BPB's, which it will not mount either. Both used to
// need NHD512 on the PC; now the volume is shown as 512-byte blocks and the
// file keeps its own layout. FAT32 has no 16-bit FAT size and is left as is.
static void nhd_translate(void) {
    uint8_t sec[512];
    if (!read_at(s_vol_off, sec, sizeof(sec))) {
        return;
    }
    FdBpb v = {};
    v.bps = s_blk_size;
    v.spc = sec[13]; v.res = rd16(sec + 14); v.nfats = sec[16]; v.root = rd16(sec + 17);
    v.media = sec[21]; v.fsz = rd16(sec + 22); v.spt = rd16(sec + 24); v.heads = rd16(sec + 26);
    v.total = s_blk_count;
    if (v.fsz == 0 || v.nfats == 0 || v.nfats > 4) {
        return;
    }
    uint8_t fat0[4];
    bool mismatch = false;
    for (uint32_t f = 0; f < v.nfats; f++) {
        if (!read_at(s_vol_off + (uint64_t)(v.res + f * v.fsz) * v.bps, &fat0[f], 1)) {
            return;
        }
        mismatch |= (fat0[f] != v.media);
    }
    if (v.bps == 512 && !mismatch) {
        return;   // nothing to change: shown exactly as it is
    }
    if (!vol_translate(sec, &v, fat0)) {
        s_npatch = 0;
        ets_printf("usb_image: layout cannot be shown in 512-byte sectors; left at %u\n",
                   (unsigned)v.bps);
        return;
    }
    s_is_fd = true;
    s_fd_map = nullptr;
    s_fd_bps = v.bps;
    s_fd_secs = v.total;
    s_blk_size = 512;
    s_blk_count = v.total * (v.bps / 512);
    const size_t len = strlen(s_status);
    snprintf(s_status + len, sizeof(s_status) - len, "%s", v.bps != 512 ? " shown as 512" : ", media byte fixed");
    ets_printf("usb_image: shown as %u blocks of 512 bytes, %d bytes adjusted\n",
               (unsigned)s_blk_count, s_npatch);
}

extern "C" bool usb_image_open(void) {
    if (!sd_mount()) {
        snprintf(s_status, sizeof(s_status), "SD card not mounted");
        return false;
    }

    // Which drive was chosen, and the image the emulator has in it. The menu
    // writes both on its way here, so it is whatever was mounted when USB Mode
    // was chosen. Without a choice it is the HDD, as it always was.
    char path[64] = "";
    uint8_t drive = 2;
    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READONLY, &nh) == ESP_OK) {
        if (nvs_get_u8(nh, NVS_KEY_DRIVE, &drive) != ESP_OK || drive > 2) {
            drive = 2;
        }
        size_t len = sizeof(path);
        if (nvs_get_str(nh, k_drive_key[drive], path, &len) != ESP_OK) {
            path[0] = 0;
        }
        nvs_close(nh);
    }
    if (!path[0]) {
        snprintf(s_status, sizeof(s_status), "no image mounted in %s", k_drive_name[drive]);
        return false;
    }
    snprintf(s_name, sizeof(s_name), "%s %s", k_drive_name[drive], path);

    char full[96];
    snprintf(full, sizeof(full), "%s%s", SD_PREFIX, path);
    s_fp = fopen(full, "r+b");
    if (!s_fp) {
        s_fp = fopen(full, "rb");   // read-only is still worth having
    }
    if (!s_fp) {
        snprintf(s_status, sizeof(s_status), "cannot open %s", path);
        return false;
    }
    ets_printf("usb_image: opened %s\n", full);

    if (drive != 2) {
        return open_fd(path);
    }

    // ---- the .NHD container --------------------------------------------
    uint8_t hdr[512];
    if (!read_at(0, hdr, sizeof(hdr)) || memcmp(hdr, "T98HDDIMAGE.R0", 14) != 0) {
        snprintf(s_status, sizeof(s_status), "not an NHD image");
        return false;
    }
    const uint32_t headersize = rd32(hdr + 272);
    const uint32_t cylinders  = rd32(hdr + 276);
    const uint16_t surfaces   = rd16(hdr + 280);
    const uint16_t sectors    = rd16(hdr + 282);
    const uint16_t secsize    = rd16(hdr + 284);
    ets_printf("usb_image: NHD hdr=%u C/H/S=%u/%u/%u sectorsize=%u\n",
               (unsigned)headersize, (unsigned)cylinders, (unsigned)surfaces,
               (unsigned)sectors, (unsigned)secsize);
    if (!headersize || !cylinders || !surfaces || !sectors || !secsize) {
        snprintf(s_status, sizeof(s_status), "NHD header not usable");
        return false;
    }

    // ---- the PC-98 partition table, at disk sector 1 --------------------
    static uint8_t ptbl[512];
    uint64_t first_cand = 0;      // where the table said the first volume was
    memset(ptbl, 0, sizeof(ptbl));
    if (read_at((uint64_t)headersize + secsize, ptbl, sizeof(ptbl))) {
        for (int i = 0; i < 16; i++) {
            const uint8_t *e = ptbl + i * 32;
            // A PC-98 partition entry, laid out as np2kai's own sxsihdd.c
            // reads it:
            //   0 mid, 1 sid, 2-3 unused, 4 ipl_sct, 5 ipl_head, 6-7 ipl_cyl,
            //   8 sector, 9 head, 10-11 cylinder, 12 end_sector, 13 end_head,
            //   14-15 end_cylinder, 16-31 name.
            // Reading the start CHS from 6/7/8 picked up the IPL's own address
            // instead, which points at the partition table itself.
            const uint8_t sid = e[1];
            const uint8_t s_sct = e[8];
            const uint8_t s_hd  = e[9];
            const uint16_t s_cyl = rd16(e + 10);
            const uint16_t e_cyl = rd16(e + 14);
            if (sid == 0 && s_cyl == 0 && e_cyl == 0) {
                continue;   // unused entry
            }
            if (s_cyl >= cylinders || s_hd >= surfaces || s_sct >= sectors) {
                continue;   // not a CHS this geometry can hold
            }
            const uint64_t lba = ((uint64_t)s_cyl * surfaces + s_hd) * sectors + s_sct;
            char what[48];
            snprintf(what, sizeof(what), "partition %d (sid %02x, C/H/S %u/%u/%u)",
                     i, sid, (unsigned)s_cyl, (unsigned)s_hd, (unsigned)s_sct);
            if (!first_cand) {
                first_cand = (uint64_t)headersize + lba * secsize;
            }
            if (try_volume((uint64_t)headersize + lba * secsize, what)) {
                snprintf(s_status, sizeof(s_status), "partition %d, %u-byte sectors",
                         i, (unsigned)s_blk_size);
                goto found;
            }
        }
    }

    // Some images are a bare volume with no partition table at all.
    if (try_volume(headersize, "sector 0")) {
        snprintf(s_status, sizeof(s_status), "unpartitioned, %u-byte sectors",
                 (unsigned)s_blk_size);
        goto found;
    }

    // Still nothing, so the table did not decode the way it was read. Rather
    // than give up on a layout assumption, go and find the volume: a DOS
    // partition starts on a cylinder boundary, and there are only `cylinders`
    // of those to look at. The BPB check is what makes this safe - it is the
    // same test the table path had to pass.
    {
        const uint32_t per_cyl = (uint32_t)surfaces * sectors;
        ets_printf("usb_image: table did not decode; scanning %u cylinder boundaries\n",
                   (unsigned)cylinders);
        for (uint32_t c = 0; c < cylinders; c++) {
            char what[40];
            snprintf(what, sizeof(what), "cylinder %u", (unsigned)c);
            if (try_volume((uint64_t)headersize + (uint64_t)c * per_cyl * secsize, what)) {
                snprintf(s_status, sizeof(s_status), "cylinder %u, %u-byte sectors",
                         (unsigned)c, (unsigned)s_blk_size);
                goto found;
            }
        }
    }

    // Nothing anywhere. Show what the first two sectors actually hold, which
    // is the only way to work out what this image is from here.
    {
        uint8_t sec[512];
        if (read_at(headersize, sec, sizeof(sec))) {
            dump("disk sector 0 (IPL), first 64 bytes:", sec, 64);
        }
        dump("disk sector 1 (partition table), first 128 bytes:", ptbl, 128);
        if (first_cand && read_at(first_cand, sec, sizeof(sec))) {
            dump("first partition's boot sector, first 64 bytes:", sec, 64);
        }
    }
    snprintf(s_status, sizeof(s_status), "no FAT volume found in the image");
    ets_printf("usb_image: no FAT volume anywhere in the image\n");
    return false;

found:
    nhd_translate();
    return true;
}

extern "C" int32_t usb_image_read(uint32_t lba, void *buf, uint32_t bytes) {
    if (!usb_image_ready()) {
        return -1;
    }
    if (s_is_fd) {
        return fd_read(lba, (uint8_t *)buf, bytes);
    }
    const uint64_t off = s_vol_off + (uint64_t)lba * s_blk_size;
    if (!read_at(off, buf, bytes)) {
        return -1;
    }
    return (int32_t)bytes;
}

extern "C" int32_t usb_image_write(uint32_t lba, const void *buf, uint32_t bytes) {
    if (!usb_image_ready()) {
        return -1;
    }
    if (s_is_fd) {
        return fd_write(lba, (const uint8_t *)buf, bytes);
    }
    const uint64_t off = s_vol_off + (uint64_t)lba * s_blk_size;
    if (fseek(s_fp, (long)off, SEEK_SET) != 0) {
        return -1;
    }
    if (fwrite(buf, 1, bytes, s_fp) != bytes) {
        return -1;
    }
    return (int32_t)bytes;
}

extern "C" void usb_image_flush(void) {
    if (s_fp) {
        fflush(s_fp);
        fsync(fileno(s_fp));
    }
}
