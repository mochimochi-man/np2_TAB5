/* ESP32 shim: dosio.h (under __LIBRETRO__) does `#include "file_stream.h"` then
 * `typedef RFILE * FILEH;`. We provide an opaque RFILE; the real file I/O is the
 * SD-backed dosio in the app (main/dosio_sd.cpp). */
#ifndef NP2_ESP32_FILE_STREAM_SHIM_H
#define NP2_ESP32_FILE_STREAM_SHIM_H
typedef struct RFILE RFILE;
#endif
