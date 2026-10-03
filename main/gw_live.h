#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  c, h, r, n;
    uint8_t  id_ok;
    uint8_t  data_ok;
    uint8_t  deleted;
    uint8_t  fm;
    uint8_t  seen;
    uint8_t  no_data;
    uint8_t  dam_gap;
    uint16_t len;
    uint32_t off;

    uint32_t pos_us;
    uint32_t rev_us;
} gw_sector_t;

#define GW_SECT_MAX_LEN 2048

int gw_live_count(void);

bool gw_live_begin(int unit);

int gw_live_port(int unit);

uint32_t gw_live_rest_count(int unit);
const char *gw_live_serial(int unit);

void gw_live_end(int unit);

void gw_live_keepalive(void);

int gw_live_read_track(int unit, int cyl, int head, int revs,
                       gw_sector_t *out, int max_out,
                       uint8_t *data, size_t data_cap);

int gw_live_read_diag(int unit, int cyl, int head, int ncode,
                      uint8_t *out, size_t cap, int *crc_bad);

#ifdef __cplusplus
}
#endif
