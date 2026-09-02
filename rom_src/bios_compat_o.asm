; OPEN-COMPAT PC98 BIOS - compatible boot ROM for np2 espresso.
;
; Reconstructed from BIOS_Compatible_O.ROM, whose source was not kept, and then
; corrected. Two things changed from the image it came out of:
;
;   1. It is built to fill E8000h-FFFFFh (0x18000 bytes) rather than only
;      F8000h-FFFFFh (0x8000). np2kai loads the BIOS with
;
;          biosrom = (file_read(fh, mem + 0x0e8000, 0x18000) == 0x18000);
;
;      so a 32KB file is rejected outright and the emulator quietly falls back
;      to its own built-in BIOS. The original therefore never ran at all - it
;      only looked as though it did, because np2's fallback boots DOS too.
;
;   2. The reset vector had a zero segment (see below).
;
; Assembling with -DORIGINAL reproduces the shipped 32KB image byte for byte,
; which is how build_compat_o.sh proves the reconstruction is still faithful.
;
; WHY THERE IS SO LITTLE HERE
;
; This does not emulate a PC-9801 BIOS. np2kai is built with BIOS_IO_EMULATION,
; so the emulator answers BIOS calls itself, in C, at the I/O level. What a ROM
; image still has to provide is only what the CPU touches directly before any of
; that takes over: a reset vector, an interrupt vector table pointing somewhere
; valid, and the hand-off to the boot sector. Roughly 190 bytes. The rest of the
; window is erased 0FFh, exactly as a real ROM reads where nothing was
; programmed.
;
; MEMORY MAP (default build)
;   E8000h-FFFFFh   the ROM window np2kai fills (0x18000 bytes)
;   F8000h          entry point         -> file offset 10000h
;   FFFC0h          signature           -> file offset 17FC0h
;   FFFF0h          reset vector        -> file offset 17FF0h
;   FFFFFh          checksum byte       -> file offset 17FFFh
;
; Build: build_compat_o.sh   (nasm alone cannot compute the checksum)

        BITS 16

%ifdef ORIGINAL
ROM_BASE        equ 0F8000h              ; the shipped image: 32KB
ROM_SIZE        equ 008000h
%else
ROM_BASE        equ 0E8000h              ; what np2kai actually wants: 96KB
ROM_SIZE        equ 018000h
%endif

CODE_BASE       equ 0F8000h              ; the code lives here either way
CODE_SEG        equ 0F800h
CODE_OFF        equ CODE_BASE - ROM_BASE ; ...at this offset into the file

        ORG 0

; Everything below F8000h is unprogrammed. On the 32KB build this is nothing.
        times CODE_OFF db 0FFh

; ---------------------------------------------------------------------------
; Entry, at F800:0000. The reset vector jumps here.
; ---------------------------------------------------------------------------
entry:
        cli
        xor ax, ax
        mov ds, ax
        mov es, ax                  ; the IVT lives at 0000:0000
        mov bx, CODE_SEG            ; segment every handler below is in

; Each vector is two words: offset then segment. Taking the segment from BX
; rather than an immediate is what keeps this compact - it is the same value
; nine times over. Labels are file offsets, so CODE_OFF comes back off to give
; the offset within CODE_SEG.
%macro SETVEC 2                     ; %1 = interrupt number, %2 = handler
        mov word [%1 * 4], %2 - CODE_OFF
        mov [%1 * 4 + 2], bx
%endmacro

        SETVEC 010h, int10_stub     ; video
        SETVEC 013h, int13_stub     ; disk
        SETVEC 016h, int16_stub     ; keyboard
        SETVEC 018h, int18_stub
        SETVEC 019h, int19_boot     ; bootstrap - the only one that does work
        SETVEC 01Ah, int1a_stub     ; time of day
        SETVEC 01Bh, int1b_stub
        SETVEC 01Ch, int1c_stub     ; timer tick
        SETVEC 01Fh, int1f_stub

        sti
        int 19h                     ; hand over to the boot sector

; INT 19h does not come back. If it ever does, stop rather than run on into
; whatever follows.
halt_loop:
        hlt
        jmp short halt_loop

; ---------------------------------------------------------------------------
; Handlers.
;
; Not implementations - somewhere safe to land. The emulator services the real
; calls at the I/O level, so all that matters here is that an unexpected INT
; returns cleanly and with a plausible result.
; ---------------------------------------------------------------------------
int10_stub:
        iret

int13_stub:
        mov ah, 1                   ; "bad command" - a failure callers expect
        stc
        iret

int16_stub:
        xor ax, ax                  ; no key waiting
        clc
        iret

int18_stub:
        iret
int1a_stub:
        iret
int1b_stub:
        iret
int1c_stub:
        iret
int1f_stub:
        iret

; The bootstrap. The boot sector is already at 0000:7C00 by the time anything
; reaches here, so this is a hand-off, not a load.
int19_boot:
        jmp 0000h:7C00h

; ---------------------------------------------------------------------------
; Erased space, then the tail.
; ---------------------------------------------------------------------------
        times (CODE_OFF + 07FC0h) - ($ - $$) db 0FFh

signature:
        db 'OPEN-COMPAT PC98 BIOS', 0

        times (CODE_OFF + 07FF0h) - ($ - $$) db 0FFh

; Reset vector at FFFF0h. The CPU starts here.
;
; The image this was recovered from had:
;
;       EA 00 00 00 00      jmp 0000:0000       <- segment zero
;       F8                  (clc - a stray byte nothing reaches)
;
; which goes to 0000:0000 instead of to the entry point. It never showed,
; because np2kai sets CS:IP itself on reset rather than fetching from FFFF0h -
; and because the ROM was the wrong size to be loaded in the first place.
; Anything that does use the vector would not have got past it.
%ifdef ORIGINAL
        db 0EAh, 000h, 000h, 000h, 000h
        db 0F8h
%else
        jmp CODE_SEG:(entry - CODE_OFF)
%endif

        times (ROM_SIZE - 1) - ($ - $$) db 0FFh

; Checksum: the whole image sums to a multiple of 256. Filled in by
; build_compat_o.sh, because nasm cannot see its own output.
        db 000h
