// The RX8130CE real-time clock on the Tab5 I2C bus.
//
// The BSP does not touch this chip, so nothing in the firmware knew the date -
// but the hardware is there, backed by the same battery as the rest of the
// board. That matters for anything the firmware writes to the SD card: FatFs
// takes its timestamps straight from time(NULL) (get_fattime in diskio.c), so
// setting the system clock once at boot gives every file a real date without
// any of the writing code knowing about it. Dumps can then be named by a plain
// counter and still sort by date on a PC.
//
// Setting the clock needs a time source, and the Tab5 has no keyboard of its
// own at boot. So: drop a file called SETTIME.TXT in the root of the SD card
// holding one line,
//
//     2026-09-02 21:30:00
//
// and the next boot adopts it and renames the file to SETTIME.OLD so it is not
// applied again. The chip is battery-backed, so this is a once-ever chore.
//
// Registers, from the Epson datasheet - note these start at 0x10, not 0x00 as
// on the older RX8025:
//
//   0x10 SEC  0x11 MIN  0x12 HOUR  0x13 WEEK  0x14 DAY  0x15 MONTH  0x16 YEAR
//   0x1D FLAG     bit1 = VLF, set when the chip has lost power and the time in
//                 it is meaningless until someone writes a new one
//   0x1E CTRL0    bit0 = STOP, held while the time registers are rewritten

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "bsp/m5stack_tab5.h"

extern "C" int ets_printf(const char *fmt, ...);

#define RX8130_ADDR 0x32
#define REG_SEC     0x10
#define REG_FLAG    0x1D
#define REG_CTRL0   0x1E

#define SETTIME_PATH "/sd/SETTIME.TXT"
#define SETTIME_DONE "/sd/SETTIME.OLD"

static i2c_master_dev_handle_t s_dev = nullptr;

static bool rtc_open(void) {
    if (s_dev) {
        return true;
    }
    if (bsp_i2c_init() != ESP_OK) {
        return false;
    }
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return false;
    }
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = RX8130_ADDR;
    cfg.scl_speed_hz = 100000;
    return i2c_master_bus_add_device(bus, &cfg, &s_dev) == ESP_OK;
}

static bool rtc_read(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len,
                                       pdMS_TO_TICKS(100)) == ESP_OK;
}

static bool rtc_write(uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t tmp[10];
    if (len + 1 > sizeof(tmp)) {
        return false;
    }
    tmp[0] = reg;
    memcpy(tmp + 1, buf, len);
    return i2c_master_transmit(s_dev, tmp, len + 1, pdMS_TO_TICKS(100)) == ESP_OK;
}

static inline int  from_bcd(uint8_t v) { return (v >> 4) * 10 + (v & 0x0f); }
static inline uint8_t to_bcd(int v)    { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// Reads the clock. Returns false when the chip is absent or has lost power,
// in which case the values are not to be believed.
extern "C" bool rtc_tab5_get(int *year, int *mon, int *day,
                             int *hour, int *min, int *sec) {
    if (!rtc_open()) {
        return false;
    }
    uint8_t flag = 0;
    if (!rtc_read(REG_FLAG, &flag, 1)) {
        return false;
    }
    uint8_t t[7];
    if (!rtc_read(REG_SEC, t, sizeof(t))) {
        return false;
    }
    const int mo = from_bcd(t[5] & 0x1f);
    const int dy = from_bcd(t[4] & 0x3f);
    if (year) *year = 2000 + from_bcd(t[6]);
    if (mon)  *mon  = mo;
    if (day)  *day  = dy;
    if (hour) *hour = from_bcd(t[2] & 0x3f);
    if (min)  *min  = from_bcd(t[1] & 0x7f);
    if (sec)  *sec  = from_bcd(t[0] & 0x7f);
    // VLF says the chip lost power; a zero month says it was never written at
    // all, which is what a factory-fresh board reads back.
    return (flag & 0x02) == 0 && mo >= 1 && mo <= 12 && dy >= 1 && dy <= 31;
}

extern "C" bool rtc_tab5_set(int year, int mon, int day,
                             int hour, int min, int sec) {
    if (!rtc_open()) {
        return false;
    }
    // Day of week is not used by anything here, but the chip keeps it and a
    // wrong value would be visible to anything else that reads the clock.
    struct tm tmv = {};
    tmv.tm_year = year - 1900;
    tmv.tm_mon = mon - 1;
    tmv.tm_mday = day;
    tmv.tm_hour = hour;
    tmv.tm_min = min;
    tmv.tm_sec = sec;
    time_t tt = mktime(&tmv);
    struct tm norm;
    localtime_r(&tt, &norm);

    uint8_t ctrl = 0;
    rtc_read(REG_CTRL0, &ctrl, 1);
    const uint8_t stopped = (uint8_t)(ctrl | 0x01);   // STOP while rewriting
    if (!rtc_write(REG_CTRL0, &stopped, 1)) {
        return false;
    }
    const uint8_t t[7] = {
        to_bcd(sec), to_bcd(min), to_bcd(hour),
        (uint8_t)(1u << (norm.tm_wday & 7)),          // WEEK is a bit, not a count
        to_bcd(day), to_bcd(mon), to_bcd(year % 100),
    };
    const bool ok = rtc_write(REG_SEC, t, sizeof(t));

    uint8_t flag = 0;
    if (rtc_read(REG_FLAG, &flag, 1)) {
        flag &= (uint8_t)~0x02;                       // the time is good now
        rtc_write(REG_FLAG, &flag, 1);
    }
    const uint8_t running = (uint8_t)(ctrl & (uint8_t)~0x01);
    rtc_write(REG_CTRL0, &running, 1);
    return ok;
}

// Adopts SETTIME.TXT if the card carries one. Returns true if the clock was
// changed.
static bool apply_settime_file(void) {
    FILE *f = fopen(SETTIME_PATH, "r");
    if (!f) {
        return false;
    }
    char line[64] = {0};
    const bool got = fgets(line, sizeof(line), f) != nullptr;
    fclose(f);

    int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
    if (!got || sscanf(line, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) < 5 ||
        y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31) {
        ets_printf("rtc: SETTIME.TXT is not YYYY-MM-DD HH:MM:SS - ignored\n");
        return false;
    }
    if (!rtc_tab5_set(y, mo, d, h, mi, s)) {
        ets_printf("rtc: could not write the clock\n");
        return false;
    }
    ets_printf("rtc: clock set from SETTIME.TXT to %04d-%02d-%02d %02d:%02d:%02d\n",
               y, mo, d, h, mi, s);
    // Renaming rather than deleting leaves a trace of what was applied, and
    // stops the same stale time being reapplied on every boot.
    remove(SETTIME_DONE);
    if (rename(SETTIME_PATH, SETTIME_DONE) != 0) {
        remove(SETTIME_PATH);
    }
    return true;
}

// Call once, after the SD card is mounted. Adopts SETTIME.TXT if present, then
// pushes the clock into the system time so FatFs stamps files with it.
//
// The RTC holds local time and is handed to the system as if it were UTC, with
// no timezone set. That keeps the numbers on the files identical to the ones
// written in SETTIME.TXT, which is what someone reading a directory listing
// expects; nothing else on this board cares about real UTC.
extern "C" void rtc_tab5_sync(void) {
    if (!rtc_open()) {
        ets_printf("rtc: no RX8130CE on the I2C bus\n");
        return;
    }
    apply_settime_file();

    int y, mo, d, h, mi, s;
    if (!rtc_tab5_get(&y, &mo, &d, &h, &mi, &s)) {
        ets_printf("rtc: clock not set - put a SETTIME.TXT on the card to set it\n");
        return;
    }
    struct tm tmv = {};
    tmv.tm_year = y - 1900;
    tmv.tm_mon = mo - 1;
    tmv.tm_mday = d;
    tmv.tm_hour = h;
    tmv.tm_min = mi;
    tmv.tm_sec = s;
    struct timeval tv = { mktime(&tmv), 0 };
    settimeofday(&tv, nullptr);
    ets_printf("rtc: %04d-%02d-%02d %02d:%02d:%02d\n", y, mo, d, h, mi, s);
}
