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
// The one thing that cannot be worked around here is the volume's logical
// sector size. It is reported to the host as-is (that is the honest thing, and
// gives the best chance of a mount), but Windows in practice only accepts 512;
// a PC-98 volume formatted with 1024-byte logical sectors will be seen and
// refused. That is a property of the image, so it is printed prominently.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

extern "C" void usb_msc_log(const char *fmt, ...);
#define ets_printf usb_msc_log
extern "C" bool sd_mount(void);

#define NVS_NS "pc98"
#define NVS_KEY_HDD "hdd"

// What the emulator's own disk paths look like: "/HDD.NHD" is relative to the
// SD root, which is mounted at /sd.
#define SD_PREFIX "/sd"

static FILE *s_fp = nullptr;
static uint64_t s_vol_off = 0;    // byte offset of the volume inside the file
static uint32_t s_blk_size = 512; // the volume's logical sector size
static uint32_t s_blk_count = 0;  // volume size in those sectors
static char s_status[64] = "not opened";
static char s_name[40] = "";

extern "C" const char *usb_image_status(void) { return s_status; }
extern "C" const char *usb_image_name(void) { return s_name; }
extern "C" uint32_t usb_image_block_count(void) { return s_blk_count; }
extern "C" uint16_t usb_image_block_size(void) { return (uint16_t)s_blk_size; }
extern "C" bool usb_image_ready(void) { return s_fp != nullptr && s_blk_count > 0; }

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
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
    if (sec[510] != 0x55 || sec[511] != 0xAA) {
        return false;
    }
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
    s_vol_off = off;
    s_blk_size = bps;
    s_blk_count = total;
    ets_printf("usb_image: %s -> FAT volume, %u sectors of %u bytes (%u MB)\n",
               what, (unsigned)total, (unsigned)bps,
               (unsigned)(((uint64_t)total * bps) >> 20));
    return true;
}

extern "C" bool usb_image_open(void) {
    if (!sd_mount()) {
        snprintf(s_status, sizeof(s_status), "SD card not mounted");
        return false;
    }

    // Which image the emulator has in its HDD slot. The menu writes this on
    // every change, so it is whatever was mounted when USB Mode was chosen.
    char path[64] = "";
    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READONLY, &nh) == ESP_OK) {
        size_t len = sizeof(path);
        if (nvs_get_str(nh, NVS_KEY_HDD, path, &len) != ESP_OK) {
            path[0] = 0;
        }
        nvs_close(nh);
    }
    if (!path[0]) {
        snprintf(s_status, sizeof(s_status), "no HDD image mounted");
        return false;
    }
    snprintf(s_name, sizeof(s_name), "%s", path);

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
    memset(ptbl, 0, sizeof(ptbl));
    if (read_at((uint64_t)headersize + secsize, ptbl, sizeof(ptbl))) {
        for (int i = 0; i < 16; i++) {
            const uint8_t *e = ptbl + i * 32;
            const uint8_t sid = e[1];
            const uint8_t s_sct = e[6];
            const uint8_t s_hd  = e[7];
            const uint16_t s_cyl = rd16(e + 8);
            const uint16_t e_cyl = rd16(e + 12);
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
    }
    snprintf(s_status, sizeof(s_status), "no FAT volume found in the image");
    ets_printf("usb_image: no FAT volume anywhere in the image\n");
    return false;

found:
    if (s_blk_size != 512) {
        ets_printf("usb_image: WARNING - this volume uses %u-byte logical sectors.\n"
                   "           Windows only mounts 512, so it will very likely be\n"
                   "           seen and refused. The image needs rebuilding with\n"
                   "           512-byte sectors.\n", (unsigned)s_blk_size);
    }
    return true;
}

extern "C" int32_t usb_image_read(uint32_t lba, void *buf, uint32_t bytes) {
    if (!usb_image_ready()) {
        return -1;
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
