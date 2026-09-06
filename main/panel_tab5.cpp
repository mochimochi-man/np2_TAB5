// MIPI-DSI panel bring-up for the M5Stack Tab5, done here rather than through
// bsp_display_new().
//
// WHY NOT THE BSP: the Tab5 ships with more than one panel. This board's is an
// ST7121, identified by reading the TOUCH controller's firmware version over
// I2C (1 = ST7121, 3 = ST7123) - M5's own driver does the same. The BSP knows
// only ILI9881C and ST7123 and picks between them by BOARD revision, so on a
// revision-2 board fitted with an ST7121 it sends the ST7123 sequence at the
// wrong lane rate and the wrong pixel clock and the panel never comes up:
// backlit, every API call returning ESP_OK, and permanently black.
//
//   ST7121   900 Mbps, 70MHz DPI, hsync  40/2/40,  vsync 24/20/200
//   ST7123  1000 Mbps, 70MHz DPI, hsync  40/2/40,  vsync  8/2/220
//   ILI9881C 1000 Mbps, 60MHz DPI, hsync 140/40/40, vsync 20/4/20
//
// (all 2 data lanes)
//
// Almost nothing is shared: the lane rate, the vertical timings and the entire
// command set all differ. But the DELIVERY is now the same for both of the
// ST712x panels: each vendor's register data, sent by the hand-rolled sequence
// below rather than through the vendor's own driver. That is not a preference.
// Both panels were black through their vendor drivers and both came up through
// this sequence, on real hardware, at the BSP's own lane rate - the ST7121
// here, the ST7123 on a user's unit through a ten-configuration sweep. See
// bringup_st7123() for what that sweep said. The earliest boards' ILI9881C is
// still driven by its vendor driver, and is still the one panel nothing has
// ever been confirmed on.
//
// The command bytes below are identical to Espressif's esp_lcd_st7121
// component, but the surrounding sequence is not, and the difference matters
// on this unit: driving the panel through esp_lcd_new_panel_st7121() leaves it
// black at every lane rate from 900 to 1300 Mbps, while the sequence here
// brings a picture up. What that driver does differently is send SWRESET
// first, wait 5 ms between every command, wait 120 ms after SLPOUT and none
// after DISPON, and never send MADCTL or COLMOD.
//
// This file deliberately keeps the BSP headers to itself. bsp/m5stack_tab5.h
// drags in esp_vfs_fat.h, whose ff.h typedefs TCHAR as char while np2kai's
// compiler_base.h typedefs it as wchar_t, so no translation unit can include
// both. lcd_mipi.cpp needs np2kai (for the CGROM); this one needs the BSP.

#include <string.h>

#include "esp_lcd_st7123.h"
#include "st7123_init_data.h"
#include "esp_lcd_ili9881c.h"
#include "ili9881c_init_data.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "driver/i2c_master.h"
#include "bsp/esp-bsp.h"
#include "hal/mipi_dsi_host_ll.h"
#include "soc/mipi_dsi_host_struct.h"
#include "nvs.h"
#include "nvs_flash.h"

extern "C" int ets_printf(const char *fmt, ...);

#define PANEL_H_RES 720
#define PANEL_V_RES 1280

// MIPI D-PHY supply and lane count. Fixed; the rest is under test.
#define DSI_LDO_CHAN 3
#define DSI_LDO_MV 2500
#define DSI_LANES 2

// ---- link configuration sweep ---------------------------------------------
// Three things about this link are not documented for this board and every
// source disagrees, so they are tried on the hardware:
//
//   lane rate    esp_lcd_st7121 says 1300, M5GFX says 900
//   burst type   esp_lcd hardcodes BURST_WITH_SYNC_PULSES; a panel that does
//                not do burst shows a correct but unstable picture
//   clock lane   esp_lcd hands it to the host's automatic control, which drops
//                the HS clock to LP during blanking - and this panel's vertical
//                front porch is 200 lines, so that is a long gap every frame
//
// One configuration per boot, the index kept in NVS and advanced each time.
// Sweeping them all inside one boot does not work: tearing the DSI bus down and
// building it again does not leave the host in the state a fresh boot does, and
// only the first step in such a run is trustworthy.
//
// The number of white squares across the top of the test pattern is the 1-based
// index, so the configuration on screen can be read off it.
#define PANEL_CFG_SWEEP 0
// Seconds to hold the static pattern before the emulator starts. 0 = off.
#define PANEL_STATIC_TEST 0

struct link_cfg_t {
    uint32_t mbps;
    mipi_dsi_ll_video_burst_type_t burst;
    bool clk_hs;   // true = clock lane forced to stay in HS
    uint32_t pclk; // DPI pixel clock, MHz
    bool no_lp;    // esp_lcd_dpi_panel_config_t.flags.disable_lp
    bool frame_bta;// host asks the panel to acknowledge every frame
};

// Round 1 swept the lane rate, the burst type and the clock lane: at 900 and
// 1040 Mbps the picture came up and flashed under every burst type and with
// the clock lane both forced and automatic, and at 1300 it stayed black. So
// none of those three is the cause, and 900 is the rate the panel's own init
// sequence is written for. What is left untested is the pixel clock.
// Round 2 swept the pixel clock from 60 to 80 MHz: no change either, and at
// every setting the host reported ErrContentionLP1 on a data lane continuously
// (~150 hits in 200 polls) while its ECC/CRC/timeout status stayed clean. That
// is a physical low-power-line collision, which points at the two things
// esp_lcd does by default and never exposes: it permits LP transitions in
// every blanking period, and it asks the panel to acknowledge every single
// frame - a bus turnaround per frame, which is exactly a per-frame disturbance.
#define CFG(lp, bta) {900, MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES, true, 70, (lp), (bta)}
static const link_cfg_t k_link_cfgs[] = {
    CFG(false, true),   // 1: esp_lcd's own defaults. Known: picture, flashes.
    CFG(true,  true),   // 2: no LP in blanking
    CFG(false, false),  // 3: no per-frame acknowledge
    CFG(true,  false),  // 4: neither
};
#undef CFG

// Used when the sweep is off.
#define DSI_LANE_MBPS 900
// What the BSP uses for the ST7123, which is a different panel entirely.
#define ST7123_LANE_MBPS 1000

// Command lists in M5GFX's format: a leading length byte covering the command
// plus its parameters, then the command, then the parameters. A zero ends it.
static const uint8_t st7121_init[] = {
    4,  0x60, 0x71, 0x21, 0xA2,
    4,  0x60, 0x71, 0x21, 0xA3,
    4,  0x60, 0x71, 0x21, 0xA4,
    2,  0x78, 0x21,
    2,  0x79, 0xEF,
    2,  0xA4, 0x31,
    7,  0xB7, 0x00, 0x00, 0x5F, 0x5F, 0x44, 0x1A,
    8,  0xB0, 0x22, 0x6B, 0x11, 0x89, 0x25, 0x43, 0x43,
    3,  0xBF, 0xA7, 0xA7,
    3,  0xA5, 0xF0, 0x03,
    7,  0xD7, 0x10, 0x2C, 0x14, 0x2A, 0x80, 0x80,
    8,  0x90, 0x71, 0x23, 0x5A, 0x20, 0x24, 0x11, 0x21,
    40, 0xA3, 0x80, 0x01, 0x8C, 0xFF, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x46, 0x00, 0x00, 0x1E, 0x5C, 0x1E, 0x80, 0x10, 0x00, 0x05,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00, 0x1E, 0x5C,
              0x1E, 0x80, 0x10, 0xEF, 0x58, 0x00, 0x00, 0x00, 0xFF,
    56, 0xA6, 0x0A, 0x00, 0x24, 0x71, 0x36, 0x00, 0x00, 0x00, 0x68, 0x68,
              0x91, 0xFF, 0x00, 0x24, 0x71, 0x37, 0x00, 0x00, 0x00, 0x68,
              0x68, 0x91, 0xFF, 0x00, 0x24, 0x71, 0x00, 0x00, 0x00, 0x00,
              0x68, 0x68, 0x91, 0xFF, 0x00, 0x2C, 0x71, 0x00, 0x01, 0x00,
              0x00, 0x68, 0x68, 0xFF, 0xFF, 0x00, 0x08, 0x80, 0x08, 0x80,
              0x06, 0x00, 0x00, 0x00, 0x00,
    61, 0xA7, 0x1A, 0x1A, 0xC0, 0x64, 0x40, 0x04, 0x15, 0x40, 0x00, 0x40,
              0x00, 0x68, 0x68, 0x91, 0xFF, 0x08, 0x80, 0x64, 0x40, 0x26,
              0x37, 0x40, 0x00, 0x00, 0x00, 0x68, 0x68, 0x91, 0xFF, 0x08,
              0x80, 0x64, 0x40, 0x8C, 0x9D, 0x40, 0x00, 0x00, 0x00, 0x68,
              0x68, 0x91, 0xFF, 0x08, 0x80, 0x64, 0x40, 0xAE, 0xBF, 0x00,
              0x00, 0x20, 0x00, 0x68, 0x68, 0x91, 0xFF, 0x08, 0x80, 0x79,
    45, 0xAC, 0x1D, 0x18, 0x19, 0x1D, 0x18, 0x19, 0x04, 0x1C, 0x1D, 0x08,
              0x0A, 0x10, 0x12, 0x0C, 0x0E, 0x14, 0x16, 0x00, 0x1D, 0x1D,
              0x1D, 0x1D, 0x1D, 0x18, 0x19, 0x1D, 0x18, 0x19, 0x06, 0x1C,
              0x1D, 0x09, 0x0B, 0x11, 0x13, 0x0D, 0x0F, 0x15, 0x17, 0x02,
              0x1D, 0x1D, 0x1D, 0x1D,
    26, 0xAD, 0x0C, 0x40, 0x46, 0x00, 0x07, 0x4B, 0x4B, 0xFF, 0xFF, 0xF0,
              0x40, 0x0E, 0x01, 0x07, 0x42, 0x42, 0xFF, 0xFF, 0x01, 0x00,
              0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    8,  0xAE, 0xF0, 0xFF, 0x03, 0xF0, 0xFF, 0x03, 0x00,
    18, 0xB2, 0x15, 0x19, 0x05, 0x23, 0x49, 0x2D, 0x03, 0x2E, 0x5C, 0xD2,
              0xFF, 0x10, 0x60, 0xFD, 0x20, 0xC0, 0x00,
    15, 0xE8, 0x20, 0x60, 0x04, 0x8E, 0x8E, 0x3E, 0x04, 0xDC, 0xDC, 0x3E,
              0x06, 0xFA, 0x26, 0x3E,
    3,  0x75, 0x03, 0x04,
    43, 0xE7, 0x4B, 0x00, 0x00, 0xBE, 0x4B, 0x8C, 0x20, 0x1A, 0xF0, 0x7D,
              0x14, 0x7D, 0x14, 0x7D, 0x14, 0x7D, 0x14, 0xFF, 0x00, 0x32,
              0x30, 0x73, 0x00, 0x00, 0xC8, 0x6A, 0xFF, 0x5A, 0x64, 0x38,
              0x88, 0x15, 0xB1, 0x01, 0x01, 0x64, 0x01, 0x01, 0x7C, 0xFF,
              0x1A, 0x51,
    3,  0xE1, 0x0C, 0x0C,
    4,  0xEA, 0x15, 0x00, 0x01,
    38, 0xC8, 0x00, 0x00, 0x04, 0x08, 0x10, 0x00, 0x1F, 0x01, 0x39, 0x3E,
              0x00, 0x78, 0x06, 0xE2, 0x02, 0x11, 0x33, 0x01, 0x7A, 0x0D,
              0x21, 0xC4, 0x0B, 0x19, 0x08, 0x32, 0xA0, 0x08, 0x1A, 0x0A,
              0xF3, 0x7F, 0x0E, 0xC5, 0xE8, 0x03, 0xFF,
    38, 0xC9, 0x00, 0x00, 0x04, 0x08, 0x10, 0x00, 0x1F, 0x01, 0x39, 0x3E,
              0x00, 0x78, 0x06, 0xE2, 0x02, 0x11, 0x33, 0x01, 0x7A, 0x0D,
              0x21, 0xC4, 0x0B, 0x19, 0x08, 0x32, 0xA0, 0x08, 0x1A, 0x0A,
              0xF3, 0x7F, 0x0E, 0xC5, 0xE8, 0x03, 0xFF,
    4,  0x60, 0x71, 0x21, 0x00,
    0,
};

static esp_lcd_panel_handle_t s_dpi = nullptr;
static uint16_t *s_fb = nullptr;
// The touch firmware version doubles as the panel identity, so 1 and 3 are
// not ours to choose. The oldest board has no such version to read, so it
// gets a number of our own that cannot collide with one.
#define PANEL_KIND_ILI9881C 9
static int s_panel_kind = 0;  // 1 = ST7121, 3 = ST7123, 9 = ILI9881C, 0 = unknown

// The panel's only reset is its power rail, on the I/O expander. The touch
// controller sits on the same display module and has its own rail there; it is
// cycled with the panel because the panel ID is read out of it.
static void panel_power(bool on) {
    bsp_feature_enable(BSP_FEATURE_LCD, on);
    bsp_feature_enable(BSP_FEATURE_TOUCH, on);
}

static void panel_power_cycle(void) {
    panel_power(false);
    vTaskDelay(pdMS_TO_TICKS(150));
    panel_power(true);
    vTaskDelay(pdMS_TO_TICKS(250));
}

// Read the touch controller's firmware version - the only way to tell an
// ST7121 board from an ST7123 one, and the touch controller's ADDRESS tells
// the oldest boards from both: an ILI9881C unit carries a GT911 instead, which
// answers at 0x14 (or 0x5D) and not at 0x55 at all. That is how the BSP finds
// its board revision too - it just stops there, and so cannot see that a 0x55
// answer might be an ST7121 rather than an ST7123.
static int probe_touch_fw(void) {
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return 0;
    }
    i2c_device_config_t dev = {};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = 0x55;          // ST7123-family touch
    dev.scl_speed_hz = 100000;
    i2c_master_dev_handle_t h = nullptr;
    if (i2c_master_bus_add_device(bus, &dev, &h) == ESP_OK) {
        const uint8_t reg[2] = {0, 0};
        uint8_t fw = 0;
        const esp_err_t e = i2c_master_transmit_receive(h, reg, sizeof(reg), &fw, 1, 200);
        i2c_master_bus_rm_device(h);
        if (e == ESP_OK && fw != 0) {
            return fw;                  // 1 = ST7121, 3 = ST7123
        }
    }
    // Nothing there: look for a GT911, which means the first-revision board.
    if (i2c_master_probe(bus, 0x14, 100) == ESP_OK ||
        i2c_master_probe(bus, 0x5d, 100) == ESP_OK) {
        return PANEL_KIND_ILI9881C;
    }
    return 0;
}

static esp_err_t send_list(esp_lcd_panel_io_handle_t io, const uint8_t *p) {
    while (p[0] != 0) {
        const uint8_t len = p[0];  // command + parameters
        const esp_err_t e = esp_lcd_panel_io_tx_param(io, p[1], &p[2], len - 1);
        if (e != ESP_OK) {
            ets_printf("panel: cmd %02x failed: %s\n", p[1], esp_err_to_name(e));
            return e;
        }
        p += len + 1;
    }
    return ESP_OK;
}

// One complete bring-up at a given lane rate. The panel is power-cycled first:
// the I/O expander is a separate I2C part that keeps its latch across a CPU
// reset, and neither the panel nor the touch controller has a reset line, so
// without this whatever the previous run left behind is still in the panel.
// That is what made the picture appear on some reboots and not others.
static bool bringup(const link_cfg_t &cfg,
                    esp_lcd_dsi_bus_handle_t *out_bus,
                    esp_lcd_panel_io_handle_t *out_io,
                    esp_lcd_panel_handle_t *out_panel,
                    uint16_t **out_fb) {
    *out_bus = nullptr;
    *out_io = nullptr;
    *out_panel = nullptr;
    *out_fb = nullptr;

    panel_power_cycle();

    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id = 0;
    bus_cfg.num_data_lanes = DSI_LANES;
    bus_cfg.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = cfg.mbps;
    if (esp_lcd_new_dsi_bus(&bus_cfg, out_bus) != ESP_OK) {
        ets_printf("panel: dsi bus failed at %u Mbps\n", (unsigned)cfg.mbps);
        return false;
    }

    esp_lcd_dbi_io_config_t dbi_cfg = {};
    dbi_cfg.virtual_channel = 0;
    dbi_cfg.lcd_cmd_bits = 8;
    dbi_cfg.lcd_param_bits = 8;
    if (esp_lcd_new_panel_io_dbi(*out_bus, &dbi_cfg, out_io) != ESP_OK) {
        ets_printf("panel: dbi io failed\n");
        return false;
    }

    // The DPI panel is created before the init sequence is sent, and
    // esp_lcd_panel_init() at the end is what starts the video.
    esp_lcd_dpi_panel_config_t dpi = {};
    dpi.virtual_channel = 0;
    dpi.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi.dpi_clock_freq_mhz = cfg.pclk;
    dpi.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi.num_fbs = 1;
    dpi.video_timing.h_size = PANEL_H_RES;
    dpi.video_timing.v_size = PANEL_V_RES;
    dpi.video_timing.hsync_back_porch = 40;
    dpi.video_timing.hsync_pulse_width = 2;
    dpi.video_timing.hsync_front_porch = 40;
    dpi.video_timing.vsync_back_porch = 24;
    dpi.video_timing.vsync_pulse_width = 20;
    // Shrinking this front porch is known to stop the touch panel working.
    dpi.video_timing.vsync_front_porch = 200;
    dpi.flags.use_dma2d = true;
    dpi.flags.disable_lp = cfg.no_lp;
    if (esp_lcd_new_panel_dpi(*out_bus, &dpi, out_panel) != ESP_OK) {
        ets_printf("panel: dpi panel failed\n");
        return false;
    }
    // Blank the frame buffer before the video starts. The DSI bridge paints
    // blue when its FIFO runs dry, and the first moments after
    // esp_lcd_panel_init() are exactly that - the DMA has not delivered a line
    // yet. Whatever the buffer holds is also whatever the panel shows for the
    // first frames, so it is cleared here rather than after the fact.
    {
        uint16_t *fb0 = nullptr;
        if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)&fb0) == ESP_OK && fb0) {
            memset(fb0, 0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t));
            esp_cache_msync(fb0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t),
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
    }
    // esp_lcd_new_panel_dpi() has just written VID_MODE_CFG: burst type, and
    // frame_bta_ack_en unconditionally on (it is not covered by disable_lp).
    // Both are overridden here, before esp_lcd_panel_init() turns the video on.
    mipi_dsi_host_ll_dpi_set_video_burst_type(&MIPI_DSI_HOST, cfg.burst);
    mipi_dsi_host_ll_dpi_enable_frame_ack(&MIPI_DSI_HOST, cfg.frame_bta);

    if (send_list(*out_io, st7121_init) != ESP_OK) {
        return false;
    }
    const uint8_t slpout = 0x11, dispon = 0x29;
    esp_lcd_panel_io_tx_param(*out_io, slpout, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(80));
    esp_lcd_panel_io_tx_param(*out_io, dispon, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    const uint8_t te_on = 0x00;
    esp_lcd_panel_io_tx_param(*out_io, 0x35, &te_on, 1);

    const uint8_t madctl = 0x00;
    const uint8_t colmod = 0x55;  // RGB565
    esp_lcd_panel_io_tx_param(*out_io, 0x36, &madctl, 1);
    esp_lcd_panel_io_tx_param(*out_io, 0x3A, &colmod, 1);

    uint8_t pwr = 0;
    if (esp_lcd_panel_io_rx_param(*out_io, 0x0A, &pwr, 1) == ESP_OK) {
        // D2 set = display on. 0x9c is what a healthy panel reports here.
        ets_printf("panel: %u Mbps, power mode = 0x%02x\n", (unsigned)cfg.mbps, pwr);
    }

    if (esp_lcd_panel_init(*out_panel) != ESP_OK) {
        ets_printf("panel: dpi init failed\n");
        return false;
    }

    // esp_lcd_panel_dpi.c hands the clock lane to the host's automatic control
    // when it starts the video ("switch the clock lane to high speed mode", but
    // the value passed is _AUTO), which lets the HS clock drop to LP during
    // blanking. With a 200-line vertical front porch that is a long gap every
    // frame, and a panel whose PLL does not survive it shows a correct picture
    // that will not hold still.
    if (cfg.clk_hs) {
        mipi_dsi_host_ll_set_clock_lane_state(&MIPI_DSI_HOST, MIPI_DSI_LL_CLOCK_LANE_STATE_HS);
    }
    if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)out_fb) != ESP_OK || !*out_fb) {
        ets_printf("panel: get_frame_buffer failed\n");
        return false;
    }
    return true;
}

static void teardown(esp_lcd_dsi_bus_handle_t bus,
                     esp_lcd_panel_io_handle_t io,
                     esp_lcd_panel_handle_t panel) {
    if (panel) {
        esp_lcd_panel_del(panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (bus) {
        esp_lcd_del_dsi_bus(bus);
    }
}

// Colour bars plus a horizontal ramp, with `marks` white squares along the top
// so the step can be read off the screen. Written once and left alone, so any
// movement on screen is the link failing rather than us redrawing.
static void test_pattern(uint16_t *fb, int marks) {
    for (int y = 0; y < PANEL_V_RES; y++) {
        uint16_t *row = fb + (size_t)y * PANEL_H_RES;
        for (int x = 0; x < PANEL_H_RES; x++) {
            const uint16_t bar = (uint16_t)((x / 90) & 7);
            const uint16_t r = (bar & 4) ? 31 : (uint16_t)(y * 31 / PANEL_V_RES);
            const uint16_t g = (bar & 2) ? 63 : (uint16_t)(y * 63 / PANEL_V_RES);
            const uint16_t b = (bar & 1) ? 31 : (uint16_t)(y * 31 / PANEL_V_RES);
            row[x] = (uint16_t)((r << 11) | (g << 5) | b);
        }
    }
    // The marker strip: black band, then `marks` white 60x60 squares.
    for (int y = 40; y < 140; y++) {
        memset(fb + (size_t)y * PANEL_H_RES, 0, PANEL_H_RES * sizeof(uint16_t));
    }
    for (int m = 0; m < marks && m < 10; m++) {
        const int x0 = 30 + m * 70;
        for (int y = 60; y < 120; y++) {
            uint16_t *row = fb + (size_t)y * PANEL_H_RES;
            for (int x = x0; x < x0 + 60 && x < PANEL_H_RES; x++) {
                row[x] = 0xFFFF;
            }
        }
    }
    esp_cache_msync(fb, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t),
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

// Call this first thing in app_main. A CPU reset stops the LEDC that drives
// the backlight and stops the DSI, but it does not touch the panel's power rail
// (that latch lives in the I/O expander) - so between the reset and the point
// where panel_tab5_init() gets to power-cycle the module, the panel is lit with
// no data arriving and shows the bridge's idle colour. That is the blue flash
// on reboot. Nothing can cover the ROM bootloader's own window, but this cuts
// out everything after it.
extern "C" void panel_tab5_blank_early(void) {
    bsp_i2c_init();
    bsp_display_brightness_init();
    bsp_display_brightness_set(0);
    bsp_feature_enable(BSP_FEATURE_LCD, false);
}

// ---- ST7123 ----------------------------------------------------------------
// The panel on later Tab5 units. Told apart from the ST7121 by the touch
// controller's firmware version - 3 rather than 1 - which is the only thing
// that distinguishes them, since both answer at the same I2C address.
//
// What follows is deliberately NOT the vendor driver, and the history matters.
// Until 1.0.2 this path went through esp_lcd_new_panel_st7123() with the BSP's
// lane rate, timings and register data, on the reasoning that following the
// path a vendor validates beats extending a sequence that was tuned against
// different silicon. On the first ST7123 unit this firmware ever ran on, that
// produced precisely the failure it was meant to avoid: every call returning
// ESP_OK, the backlight on, and the panel black.
//
// A ten-configuration sweep on that unit settled it (np2_TAB5_PanelDiag, one
// configuration per boot, the tester tapping whenever a picture appeared). The
// register data was never the problem; the way it was being delivered was:
//
//   BSP 1.3.0's own bsp_display_new()            black
//   vendor driver, 1000 Mbps  (what 1.0.1 did)   black
//   vendor driver,  965 Mbps                     black
//   vendor driver,  900 Mbps                     PICTURE
//   vendor driver, 1300 Mbps                     PICTURE
//   THIS SEQUENCE, 1000 Mbps                     PICTURE
//   the ST7121 sequence and its data             black
//
// The lane rate is not a smooth axis on this panel - 900 and 1300 work while
// 965 and 1000 between them do not - so a fix that moves the rate is a fix
// resting on one unit's analogue margins. This keeps the BSP's rate and changes
// the delivery instead, which is also the smaller change: it is the sequence
// the ST7121 has always used here, with the ST7123's register data in it.
//
// The difference from the vendor driver that most likely matters is its last
// line. esp_lcd hands the clock lane to the host's automatic control, which
// lets the HS clock fall to LP during blanking, and this panel's vertical front
// porch is 220 lines - a long gap, every frame, for a PLL to come back from.
// Forcing the clock lane to stay in HS is what the ST7121 needed too. The
// sweep could not separate that from the rest of the sequence (one step, one
// tester, one run), so the whole sequence is adopted rather than the one line
// guessed at.
static bool bringup_st7123(esp_lcd_dsi_bus_handle_t *out_bus,
                           esp_lcd_panel_io_handle_t *out_io,
                           esp_lcd_panel_handle_t *out_panel,
                           uint16_t **out_fb) {
    *out_bus = nullptr;
    *out_io = nullptr;
    *out_panel = nullptr;
    *out_fb = nullptr;

    panel_power_cycle();

    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id = 0;
    bus_cfg.num_data_lanes = DSI_LANES;
    bus_cfg.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = ST7123_LANE_MBPS;
    if (esp_lcd_new_dsi_bus(&bus_cfg, out_bus) != ESP_OK) {
        ets_printf("panel: dsi bus failed at %u Mbps\n", (unsigned)ST7123_LANE_MBPS);
        return false;
    }

    esp_lcd_dbi_io_config_t dbi_cfg = {};
    dbi_cfg.virtual_channel = 0;
    dbi_cfg.lcd_cmd_bits = 8;
    dbi_cfg.lcd_param_bits = 8;
    if (esp_lcd_new_panel_io_dbi(*out_bus, &dbi_cfg, out_io) != ESP_OK) {
        ets_printf("panel: dbi io failed\n");
        return false;
    }

    // The BSP's timings for this panel, unchanged. Only the delivery differs.
    esp_lcd_dpi_panel_config_t dpi = {};
    dpi.virtual_channel = 0;
    dpi.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi.dpi_clock_freq_mhz = 70;
    dpi.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi.num_fbs = 1;
    dpi.video_timing.h_size = PANEL_H_RES;
    dpi.video_timing.v_size = PANEL_V_RES;
    dpi.video_timing.hsync_back_porch = 40;
    dpi.video_timing.hsync_pulse_width = 2;
    dpi.video_timing.hsync_front_porch = 40;
    dpi.video_timing.vsync_back_porch = 8;
    dpi.video_timing.vsync_pulse_width = 2;
    dpi.video_timing.vsync_front_porch = 220;
    dpi.flags.use_dma2d = true;
    if (esp_lcd_new_panel_dpi(*out_bus, &dpi, out_panel) != ESP_OK) {
        ets_printf("panel: dpi panel failed\n");
        return false;
    }

    // Blank before the video starts: the DSI bridge paints blue while its FIFO
    // is empty, which is exactly the first frames after the video comes on.
    {
        uint16_t *fb0 = nullptr;
        if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)&fb0) == ESP_OK && fb0) {
            memset(fb0, 0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t));
            esp_cache_msync(fb0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t),
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
    }
    // esp_lcd_new_panel_dpi() has just written VID_MODE_CFG: the burst type, and
    // frame_bta_ack_en unconditionally on. A bus turnaround per frame is a
    // per-frame disturbance on the low-power lines; it is off on both panels.
    mipi_dsi_host_ll_dpi_set_video_burst_type(&MIPI_DSI_HOST,
                                              MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
    mipi_dsi_host_ll_dpi_enable_frame_ack(&MIPI_DSI_HOST, false);

    // The BSP's register data, sent straight through with no software reset
    // ahead of it and no five milliseconds between commands - the two things
    // the vendor driver adds. The tail of the list (MADCTL, sleep out, display
    // on, tearing line) is skipped and sent below with waits that the list does
    // not carry.
    {
        const int n = (int)(sizeof(disp_init_data_st7123) /
                            sizeof(disp_init_data_st7123[0]));
        for (int i = 0; i < n; i++) {
            const st7123_lcd_init_cmd_t *c = &disp_init_data_st7123[i];
            if (c->cmd == 0x36 || c->cmd == 0x11 || c->cmd == 0x29 || c->cmd == 0x35) {
                continue;
            }
            if (esp_lcd_panel_io_tx_param(*out_io, c->cmd, c->data, c->data_bytes) != ESP_OK) {
                ets_printf("panel: cmd %02x failed\n", c->cmd);
                return false;
            }
        }
    }

    const uint8_t slpout = 0x11, dispon = 0x29;
    esp_lcd_panel_io_tx_param(*out_io, slpout, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(80));
    esp_lcd_panel_io_tx_param(*out_io, dispon, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    const uint8_t te_on = 0x00;
    esp_lcd_panel_io_tx_param(*out_io, 0x35, &te_on, 1);

    const uint8_t madctl = 0x00;
    const uint8_t colmod = 0x55;  // RGB565, which the vendor list never sets
    esp_lcd_panel_io_tx_param(*out_io, 0x36, &madctl, 1);
    esp_lcd_panel_io_tx_param(*out_io, 0x3A, &colmod, 1);

    // BEFORE esp_lcd_panel_init(), and that is not incidental. A DSI read puts
    // the host into command mode (mipi_dsi_host_ll_enable_video_mode(false),
    // inside the HAL) and nothing puts it back except dpi_panel_init(). Read
    // after the video has started and the host stays in command mode.
    uint8_t pwr = 0;
    if (esp_lcd_panel_io_rx_param(*out_io, 0x0A, &pwr, 1) == ESP_OK) {
        // D2 set = display on. 0x9c is what a healthy panel reports here.
        ets_printf("panel: ST7123 power mode = 0x%02x%s\n", pwr,
                   (pwr & 0x04) ? "" : "  (display reports OFF)");
    } else {
        ets_printf("panel: ST7123 did not answer the power-mode read\n");
    }

    if (esp_lcd_panel_init(*out_panel) != ESP_OK) {
        ets_printf("panel: dpi init failed\n");
        return false;
    }

    // esp_lcd_panel_dpi.c hands the clock lane to the host's automatic control
    // when it starts the video, which lets the HS clock drop to LP during
    // blanking. With a 220-line vertical front porch that is a long gap every
    // frame. This is the line the ST7121 needed, and the most likely reason
    // this sequence comes up where the vendor driver does not.
    mipi_dsi_host_ll_set_clock_lane_state(&MIPI_DSI_HOST, MIPI_DSI_LL_CLOCK_LANE_STATE_HS);

    if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)out_fb) != ESP_OK || !*out_fb) {
        ets_printf("panel: no frame buffer\n");
        return false;
    }
    return true;
}

// ---- ILI9881C --------------------------------------------------------------
// The panel on the first Tab5 revision, which also carries a GT911 touch
// controller rather than the ST7123-family one. UNTESTED here for the same
// reason as the ST7123 path, and built the same way: the vendor driver, the
// BSP's register data, the BSP's timings.
//
// It differs from both of the others in more than timing - 60MHz pixel clock
// against 70, and a horizontal blanking nearly three times as wide - and its
// vendor config wants the lane count, which the ST7123 one does not.
static bool bringup_ili9881c(esp_lcd_dsi_bus_handle_t *out_bus,
                             esp_lcd_panel_io_handle_t *out_io,
                             esp_lcd_panel_handle_t *out_panel,
                             uint16_t **out_fb) {
    *out_bus = nullptr;
    *out_io = nullptr;
    *out_panel = nullptr;
    *out_fb = nullptr;

    panel_power_cycle();

    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id = 0;
    bus_cfg.num_data_lanes = DSI_LANES;
    bus_cfg.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = ST7123_LANE_MBPS;   // the BSP uses 1000 for both
    if (esp_lcd_new_dsi_bus(&bus_cfg, out_bus) != ESP_OK) {
        ets_printf("panel: dsi bus failed at %u Mbps\n", (unsigned)ST7123_LANE_MBPS);
        return false;
    }

    esp_lcd_dbi_io_config_t dbi_cfg = {};
    dbi_cfg.virtual_channel = 0;
    dbi_cfg.lcd_cmd_bits = 8;
    dbi_cfg.lcd_param_bits = 8;
    if (esp_lcd_new_panel_io_dbi(*out_bus, &dbi_cfg, out_io) != ESP_OK) {
        ets_printf("panel: dbi io failed\n");
        return false;
    }

    esp_lcd_dpi_panel_config_t dpi = {};
    dpi.virtual_channel = 0;
    dpi.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi.dpi_clock_freq_mhz = 60;
    dpi.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi.num_fbs = 1;
    dpi.video_timing.h_size = PANEL_H_RES;
    dpi.video_timing.v_size = PANEL_V_RES;
    dpi.video_timing.hsync_back_porch = 140;
    dpi.video_timing.hsync_pulse_width = 40;
    dpi.video_timing.hsync_front_porch = 40;
    dpi.video_timing.vsync_back_porch = 20;
    dpi.video_timing.vsync_pulse_width = 4;
    dpi.video_timing.vsync_front_porch = 20;
    dpi.flags.use_dma2d = true;

    ili9881c_vendor_config_t vendor = {};
    vendor.init_cmds = disp_init_data_ili9881c;
    vendor.init_cmds_size = (uint16_t)(sizeof(disp_init_data_ili9881c) /
                                       sizeof(disp_init_data_ili9881c[0]));
    vendor.mipi_config.dsi_bus = *out_bus;
    vendor.mipi_config.dpi_config = &dpi;
    vendor.mipi_config.lane_num = DSI_LANES;

    esp_lcd_panel_dev_config_t pcfg = {};
    pcfg.reset_gpio_num = -1;          // BSP_LCD_RST is GPIO_NUM_NC on the Tab5
    pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    pcfg.bits_per_pixel = 16;
    pcfg.vendor_config = &vendor;

    if (esp_lcd_new_panel_ili9881c(*out_io, &pcfg, out_panel) != ESP_OK) {
        ets_printf("panel: esp_lcd_new_panel_ili9881c failed\n");
        return false;
    }

    {
        uint16_t *fb0 = nullptr;
        if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)&fb0) == ESP_OK && fb0) {
            memset(fb0, 0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t));
            esp_cache_msync(fb0, (size_t)PANEL_H_RES * PANEL_V_RES * sizeof(uint16_t),
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
    }
    mipi_dsi_host_ll_dpi_set_video_burst_type(&MIPI_DSI_HOST,
                                              MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
    mipi_dsi_host_ll_dpi_enable_frame_ack(&MIPI_DSI_HOST, false);

    esp_lcd_panel_reset(*out_panel);
    if (esp_lcd_panel_init(*out_panel) != ESP_OK) {
        ets_printf("panel: ili9881c init failed\n");
        return false;
    }

    uint8_t pwr = 0;
    if (esp_lcd_panel_io_rx_param(*out_io, 0x0A, &pwr, 1) == ESP_OK) {
        ets_printf("panel: ILI9881C power mode = 0x%02x%s\n", pwr,
                   (pwr & 0x04) ? "" : "  (display reports OFF)");
    } else {
        ets_printf("panel: ILI9881C did not answer the power-mode read\n");
    }

    if (esp_lcd_dpi_panel_get_frame_buffer(*out_panel, 1, (void **)out_fb) != ESP_OK) {
        ets_printf("panel: no frame buffer\n");
        return false;
    }
    return true;
}

extern "C" bool panel_tab5_init(void) {
    bsp_i2c_init();
    panel_power_cycle();

    // The touch controller answers a while after its rail comes up, so give it
    // a few tries rather than deciding the panel is unknown on the first miss.
    for (int i = 0; i < 20 && s_panel_kind == 0; i++) {
        s_panel_kind = probe_touch_fw();
        if (s_panel_kind == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    ets_printf("panel: touch probe %d -> %s\n", s_panel_kind,
               (s_panel_kind == 1) ? "ST7121" :
               (s_panel_kind == 3) ? "ST7123" :
               (s_panel_kind == PANEL_KIND_ILI9881C) ? "ILI9881C (GT911 touch)" : "unknown");
    if (s_panel_kind == 0) {
        // The touch controller never answered, so there is nothing to go on.
        // Trying the ST7121 sequence anyway beats refusing to bring the display
        // up at all, and the line above says what happened.
        ets_printf("panel: touch controller silent - assuming ST7121\n");
        s_panel_kind = 1;
    }

    // MIPI D-PHY supply. Acquired once and kept: the bus is created and
    // destroyed repeatedly during the sweep, the PHY rail is not.
    esp_ldo_channel_handle_t ldo = nullptr;
    esp_ldo_channel_config_t ldo_cfg = {};
    ldo_cfg.chan_id = DSI_LDO_CHAN;
    ldo_cfg.voltage_mv = DSI_LDO_MV;
    if (esp_ldo_acquire_channel(&ldo_cfg, &ldo) != ESP_OK) {
        ets_printf("panel: LDO acquire failed\n");
        return false;
    }

    bsp_display_brightness_init();

    // What the sweep settled on: esp_lcd defaults everywhere except the
    // per-frame acknowledge, which is what was breaking the link.
    link_cfg_t cfg = {DSI_LANE_MBPS, MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES, true, 70, false, false};
    int marks = 0;
#if PANEL_CFG_SWEEP
    const int n = (int)(sizeof(k_link_cfgs) / sizeof(k_link_cfgs[0]));
    int idx = 0;
    nvs_handle_t nvs = 0;
    if (nvs_open("pc98", NVS_READWRITE, &nvs) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(nvs, "dsicfg", &v) == ESP_OK) {
            idx = v % n;
        }
        nvs_set_u8(nvs, "dsicfg", (uint8_t)((idx + 1) % n));
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    cfg = k_link_cfgs[idx];
    marks = idx + 1;
    static const char *k_burst_name[] = {"non-burst/pulses", "non-burst/events", "burst/pulses"};
    ets_printf("panel: SWEEP config %d of %d: %u Mbps, %s, clock lane %s, pclk %u MHz (%d white squares)\n",
               idx + 1, n, (unsigned)cfg.mbps, k_burst_name[cfg.burst],
               cfg.clk_hs ? "forced HS" : "auto", (unsigned)cfg.pclk, marks);
#endif

    esp_lcd_dsi_bus_handle_t bus = nullptr;
    esp_lcd_panel_io_handle_t io = nullptr;
    const bool up = (s_panel_kind == 3)
                    ? bringup_st7123(&bus, &io, &s_dpi, &s_fb)
                    : (s_panel_kind == PANEL_KIND_ILI9881C)
                      ? bringup_ili9881c(&bus, &io, &s_dpi, &s_fb)
                      : bringup(cfg, &bus, &io, &s_dpi, &s_fb);
    if (!up) {
        return false;
    }
    // Only now light it. Between esp_lcd_panel_init() and the first delivered
    // frame the bridge shows its underrun colour, and on a reboot that reads as
    // the screen flashing blue.
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_display_backlight_on();
    ets_printf("panel: %s up at %u Mbps, %dx%d, fb=%p\n",
               (s_panel_kind == 3) ? "ST7123" :
               (s_panel_kind == PANEL_KIND_ILI9881C) ? "ILI9881C" : "ST7121",
               (unsigned)((s_panel_kind == 1) ? cfg.mbps : ST7123_LANE_MBPS),
               PANEL_H_RES, PANEL_V_RES, (void *)s_fb);

#if PANEL_STATIC_TEST
    // Paint a fixed pattern once, then leave the frame buffer completely alone.
    // Nothing writes to PSRAM from this core during the wait and the emulator
    // has not started, so anything moving on screen is the link, not us.
    test_pattern(s_fb, marks);
    ets_printf("panel: STATIC TEST - holding a fixed pattern for %d s\n", PANEL_STATIC_TEST);
    // While it holds, watch the DSI host's error status. int_st0 carries the
    // acknowledge errors the PANEL sends back over the link; int_st1 carries
    // the host's own D-PHY, ECC, CRC, packet-size and timeout errors. Reading
    // clears them, so each line is what happened in the last 200 ms. A clean
    // run here means the link is fine and the picture is unstable for a reason
    // outside the DSI.
    {
        // Reading clears, so drain whatever bring-up latched (the DCS read
        // above does a bus turnaround, which by itself sets a contention bit)
        // before counting.
        (void)MIPI_DSI_HOST.int_st0.val;
        (void)MIPI_DSI_HOST.int_st1.val;
        uint32_t or0 = 0, or1 = 0;
        int hits = 0;
        for (int i = 0; i < PANEL_STATIC_TEST * 20; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
            const uint32_t s0 = MIPI_DSI_HOST.int_st0.val;
            const uint32_t s1 = MIPI_DSI_HOST.int_st1.val;
            if (s0 | s1) {
                hits++;
                or0 |= s0;
                or1 |= s1;
            }
        }
        ets_printf("panel: dsi errors in %d polls: %d, int_st0=%08x int_st1=%08x\n",
                   PANEL_STATIC_TEST * 20, hits, (unsigned)or0, (unsigned)or1);
    }
    ets_printf("panel: STATIC TEST over, starting the emulator\n");
#endif
    return true;
}

// The USB-A host port is powered through the second I/O expander, and nothing
// else turns it on. usb_kbd.cpp cannot call bsp_feature_enable() itself (the
// BSP headers and np2kai cannot be in the same translation unit - see the top
// of this file), so it goes through here.
// Backlight level, 0-100. The LEDC timer behind it is set up in
// panel_tab5_init(); this only changes the duty.
extern "C" void tab5_backlight_set(int percent) {
    bsp_display_brightness_set(percent);
}

extern "C" void tab5_usb_host_power(bool on) {
    bsp_feature_enable(BSP_FEATURE_USB, on);
}

extern "C" uint16_t *panel_tab5_framebuffer(void) {
    return s_fb;
}

extern "C" esp_lcd_panel_handle_t panel_tab5_handle(void) {
    return s_dpi;
}
