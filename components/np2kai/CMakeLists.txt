# np2kai core as an ESP-IDF component — i286c (V30/286) CPU core build.
# Switched from the heavy i386c IA-32 interpreter to the light i286c core: VM21 is
# a V30 machine, so this is both faster and more accurate (no 386/EMM386/FPU/SSE).
# The i386c config is preserved in CMakeLists.txt.ia32.bak.
# Static .bss/.common (incl. mem[]) still routed to PSRAM via linker.lf.
set(NP2 ${CMAKE_CURRENT_SOURCE_DIR}/np2kai)

file(GLOB NP2_SRCS
    ${NP2}/i286c/*.c
    ${NP2}/io/*.c ${NP2}/io/pci/*.c
    ${NP2}/mem/*.c ${NP2}/vram/*.c ${NP2}/fdd/*.c ${NP2}/font/*.c
    ${NP2}/generic/*.c ${NP2}/lio/*.c ${NP2}/bios/*.c ${NP2}/cbus/*.c
    ${NP2}/codecnv/*.c ${NP2}/common/*.c ${NP2}/trap/*.c
    ${NP2}/sound/*.c
    ${NP2}/diskimage/*.c ${NP2}/diskimage/fd/*.c ${NP2}/diskimage/cd/*.c
)
# root glue (no debugsub386.c — that's the i386c debug helper)
list(APPEND NP2_SRCS
    ${NP2}/pccore.c ${NP2}/nevent.c ${NP2}/timing.c ${NP2}/calendar.c
    ${NP2}/keystat.c ${NP2}/statsave.c)

idf_component_register(
    SRCS ${NP2_SRCS}
    INCLUDE_DIRS
        shim
        ${NP2} ${NP2}/common ${NP2}/i286c
        ${NP2}/io ${NP2}/io/pci ${NP2}/mem
        ${NP2}/vram ${NP2}/fdd ${NP2}/font ${NP2}/generic
        ${NP2}/sound ${NP2}/sound/getsnd
        ${NP2}/codecnv ${NP2}/cbus ${NP2}/bios ${NP2}/lio ${NP2}/diskimage
        ${NP2}/diskimage/fd ${NP2}/diskimage/cd ${NP2}/trap
        ${NP2}/sdl ${NP2}/sdl/libretro ${NP2}/sdl/libretro/libretro-common/include
    LDFRAGMENTS linker.lf
    REQUIRES esp_timer
)

# i286c (V30/286) define set — the upstream "NP2" defines (NOT the NP21 IA-32 set:
# no CPUCORE_IA32, no IDEIO/PC9821, no FPU/MMX/SSE, no PAGING).
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    __LIBRETRO__ NP2_ESP32 NP2KAI_GIT_TAG="esp32" NP2KAI_GIT_HASH="0000000"
    BIOS_IO_EMULATION OSLANG_UTF8 OSLINEBREAK_LF
    SOUNDRESERVE=40
    SUPPORT_8BPP SUPPORT_16BPP SUPPORT_24BPP SUPPORT_32BPP
    SUPPORT_CRT15KHZ SUPPORT_FAST_MEMORYCHECK
    SUPPORT_HRTIMER SUPPORT_KAI_IMAGES SUPPORT_LARGE_HDD SUPPORT_NORMALDISP
    SUPPORT_RESUME
    SUPPORT_SCSI SUPPORT_SMPU98 SUPPORT_SOFTKBD=0
    SUPPORT_STATSAVE=10 SUPPORT_UTF8 SUPPORT_V30EXT SUPPORT_V30ORIGINAL
    VAEG_FIX
)

# Third-party core: suppress all warnings (-w) so IDF's -Werror can't fire, and
# explicitly disable the GCC14 categories that default to hard errors.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -w
    -Wno-incompatible-pointer-types -Wno-int-conversion -Wno-implicit-function-declaration
    -fno-strict-aliasing)
