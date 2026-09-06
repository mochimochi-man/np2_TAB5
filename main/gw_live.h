// Reading a physical floppy while the emulator runs, rather than dumping it to
// a file first.
//
// This is the seam between gw_mode.cpp, which owns the Greaseweazle and the MFM
// decoder, and fdd_gw_live.cpp, which plugs into np2kai as a disk backend. The
// two cannot be one file: the decoder side reaches for the BSP and the ESP-IDF
// USB stack, the emulator side reaches for np2kai, and those two sets of
// headers cannot be included together (ff.h and compiler_base.h disagree about
// what TCHAR is). So the boundary is this header, which includes neither.
//
// What crosses it is a track's worth of sectors exactly as they were found on
// the disk - every sector ID, whatever it says, and whether each field's CRC
// was good. That is the whole point of the exercise: a .HDM or .NFD can only
// hold a regular geometry, so writing a protected disk to one silently drops
// the sectors the protection is made of. Nothing is written to a file here, so
// nothing has to be left out.

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// One sector, as the disk actually presents it. c/h/r/n are the ID field's own
// bytes, which on a protected track need not agree with where the head is or
// with any regular numbering.
typedef struct {
    uint8_t  c, h, r, n;
    uint8_t  id_ok;      // the ID field's CRC checked out
    uint8_t  data_ok;    // the data field's CRC checked out
    uint8_t  deleted;    // it carried a deleted-data address mark (0xF8)
    uint8_t  fm;         // written in single density, not MFM
    uint8_t  seen;       // how many revolutions it was found on
    uint16_t len;        // 128 << n
    uint32_t off;        // where its data starts in the caller's buffer
} gw_sector_t;

// The largest sector this can carry. Regular PC-98 2HD sectors are 1024 bytes;
// protection sectors of 2048 (N=4) are common enough to be worth room for.
#define GW_SECT_MAX_LEN 2048

// How many Greaseweazles are plugged in. A PC-98 has two floppy drives and an
// installer that asks for disk B in drive 2 needs both, so each unit is a
// separate device - told apart by USB address, since they share a VID and PID.
int gw_live_count(void);

// Spin one drive up and leave it running. Call once when its disk is mounted.
bool gw_live_begin(int unit);

// Which of them this is, for the menu to say. The hub port is a socket that can
// be traced by its cable and does not move; the serial identifies the device
// itself. 0 and "" when the unit was never opened.
int gw_live_port(int unit);

// How many times the drive has been left to stop. Rises by one each time, and
// a disk can only be changed while it is stopped - so a change in this is the
// cue to stop trusting anything cached from before.
uint32_t gw_live_rest_count(int unit);
const char *gw_live_serial(int unit);

// Motor off, drive deselected.
void gw_live_end(int unit);

// Nudge every mounted drive so the device does not let it stop. Cheap, and
// safe to call every frame - it does something only every few seconds.
void gw_live_keepalive(void);

// Read one physical track and hand back every sector found on it. Sector data
// is written into `data` at each sector's `off`. Returns the number of sectors,
// or -1 if the track could not be read at all.
//
// Revolutions are read out and merged: a sector seen more than once keeps its
// best copy, so a marginal one still has several chances. Sectors whose data
// CRC fails are kept rather than discarded - on a protected disk that failure
// is the point, and the emulator has to be able to report it.
int gw_live_read_track(int unit, int cyl, int head, int revs,
                       gw_sector_t *out, int max_out,
                       uint8_t *data, size_t data_cap);

#ifdef __cplusplus
}
#endif
