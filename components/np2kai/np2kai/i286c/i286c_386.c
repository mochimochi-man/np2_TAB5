// The 386 extension, compiled on its own (see i286c_386.inc).
//
// Like the rest of the interpreter it runs from IRAM (linker.lf). To make room,
// the non-ISR FreeRTOS functions run from flash instead
// (CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH).

#include	<compiler.h>
#include	<cpucore.h>
#include	"i286c.h"
#include	<pccore.h>
#include	<io/iocore.h>
#include	<bios/bios.h>
#include	"i286c.mcr"

#define	MAX_PREFIX		8

#include	"i286c_386.inc"
