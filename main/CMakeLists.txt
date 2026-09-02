idf_component_register(
    SRCS "main.cpp" "dosio_sd.cpp" "platform_stubs.cpp" "lcd_mipi.cpp" "panel_tab5.cpp" "usb_kbd.cpp" "audio_codec.cpp" "menu_disk.cpp" "touch_mouse.cpp" "gw_mode.cpp" "rtc_tab5.cpp" "screenshot.cpp" "bt_hid.cpp" "usb_msc.cpp" "usb_image.cpp" "sd_tab5.cpp"
    INCLUDE_DIRS "."
    # The compatible BIOS and font, built into the image rather than read off
    # the SD card. The card is still searched first when the menu names a file,
    # but a stock unit needs no card contents at all - which is what makes an
    # M5Burner image work out of the box, since that ships a firmware image and
    # nothing else. 96KB + 282KB of flash, memory-mapped, costing no RAM.
    EMBED_FILES "rom/BIOS_Compatible_O.ROM" "rom/FONT_ESP.ROM"
    # No arduino-esp32, no bt/esp_hid: the P4 has no on-die radio (Bluetooth
    # would go through the companion ESP32-C6 over esp-hosted, which is a later
    # phase), and the handful of Arduino calls the S3 forks used have IDF
    # equivalents. espressif__m5stack_tab5 replaces the hand-rolled board_bsp.
    REQUIRES np2kai fatfs sdmmc esp_lcd usb driver nvs_flash
             espressif__m5stack_tab5 espressif__usb_host_hid espressif__tinyusb espressif__esp_lcd_st7121 espressif__esp_lcd_st7123 espressif__esp_lcd_ili9881c espressif__esp_lcd_touch
             esp_driver_i2s esp_psram esp_driver_ppa bt esp_hid esp_hosted espressif__usb_host_cdc_acm
)

# Platform files include the np2kai core headers, so they need the SAME target
# defines (so shared structs like SCRNMNG/FILEH match the precompiled core) + GCC14 flags.
# MUST match the np2kai core's define set (struct layouts like NP2CFG depend on
# SUPPORT_IDEIO etc.) — i286c/V30 build, no IA-32/IDEIO/FPU.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    __LIBRETRO__ NP2_ESP32 NP2KAI_GIT_TAG="esp32" NP2KAI_GIT_HASH="0000000"
    BIOS_IO_EMULATION OSLANG_UTF8 OSLINEBREAK_LF
    SUPPORT_8BPP SUPPORT_16BPP SUPPORT_24BPP SUPPORT_32BPP
    SUPPORT_CRT15KHZ SUPPORT_FAST_MEMORYCHECK
    SUPPORT_HRTIMER SUPPORT_KAI_IMAGES SUPPORT_LARGE_HDD SUPPORT_NORMALDISP
    SUPPORT_RESUME
    SUPPORT_SCSI SUPPORT_SMPU98 SUPPORT_SOFTKBD=0
    SUPPORT_STATSAVE=10 SUPPORT_UTF8 SUPPORT_V30EXT SUPPORT_V30ORIGINAL
    VAEG_FIX
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=incompatible-pointer-types -Wno-incompatible-pointer-types
    -Wno-error=int-conversion -Wno-error=implicit-function-declaration
    -Wno-error=format -Wno-error=format-truncation -Wno-error=return-type -fno-strict-aliasing)
