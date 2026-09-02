# np2_TAB5 — PC-98 emulator (NP2kai, i286c core) on the M5Stack Tab5 (ESP32-P4).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Building a known-good project on a newer IDF/GCC. Two defenses so newer-compiler
# warnings in dependencies don't break the build:
#  - `-Wno-error`: don't promote warnings to errors globally.
#  - `-Wno-stringop-truncation`/`-Wno-stringop-overflow`: fully disable these GCC
#    warnings so they can't fire even if a component appends its own bare `-Werror`
#    after this (a disabled warning is not re-enabled by `-Werror`). esp-modbus does this.
idf_build_set_property(COMPILE_OPTIONS "-Wno-error;-Wno-stringop-truncation;-Wno-stringop-overflow" APPEND)
project(np2_espresso)

# The raw tinyusb component finds tusb_config.h through its include path, and
# main/ is not one of its dependencies. Inject it, rather than adding main/ to
# every component's include path globally.
idf_component_get_property(_tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${_tusb_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/main")
