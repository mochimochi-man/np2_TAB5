; BIOSCHK.COM - identify the active PC-98 BIOS from DOS
;
; Build: nasm -Wall -Werror -f bin -o BIOSCHK.COM bioschk.asm
; Run:   BIOSCHK
;
; This program only reads the active ROM mapping. It does not switch ROM
; banks or modify BIOS memory, so the result describes what DOS can see.

        CPU 8086
        BITS 16
        ORG 0x100

start:
        cld
        push    cs
        pop     ds

        mov     dx, msg_title
        call    print

        ; Canonical marker: the PC-486NAV vendor text begins at F3274h.
        mov     ax, 0xf000
        mov     es, ax
        mov     di, 0x3274
        mov     si, marker
        mov     cx, marker_len
        repe    cmpsb
        je      found_vendor

        ; The 32 KiB ROM variant cannot contain F3274h, so it carries a mirror
        ; immediately below the reset-vector area.
        mov     ax, 0xf800
        mov     es, ax
        mov     di, 0x7fc0
        mov     si, marker
        mov     cx, marker_len
        repe    cmpsb
        je      found_mirror

        ; Recognise ROMs produced before the new identification was adopted.
        mov     di, 0x7fc0
        mov     si, old_marker
        mov     cx, old_marker_len
        repe    cmpsb
        je      found_old

        mov     dx, msg_other
        call    print
        mov     ax, 0x4c01
        int     0x21

found_vendor:
        mov     dx, msg_codex
        call    print
        mov     dx, msg_vendor_addr
        call    print
        mov     ax, 0x4c00
        int     0x21

found_mirror:
        mov     dx, msg_codex
        call    print
        mov     dx, msg_mirror_addr
        call    print
        mov     ax, 0x4c00
        int     0x21

found_old:
        mov     dx, msg_old
        call    print
        mov     ax, 0x4c02
        int     0x21

print:
        mov     ah, 0x09
        int     0x21
        ret

marker:
        db      'Compatible BIOS by Codex'
marker_len      equ $ - marker
old_marker:
        db      'PC98NATIVE01'
old_marker_len  equ $ - old_marker

msg_title:
        db      'BIOSCHK 1.0 - active PC-98 ROM check', 13, 10, '$'
msg_codex:
        db      'Active BIOS: Compatible BIOS by Codex', 13, 10, '$'
msg_vendor_addr:
        db      'Marker address: F3274h (PC-486NAV vendor-text slot)', 13, 10, '$'
msg_mirror_addr:
        db      'Marker address: FFFC0h (32 KiB ROM mirror)', 13, 10, '$'
msg_old:
        db      'Active BIOS: older PC98N build (PC98NATIVE01)', 13, 10
        db      'Update the ROM to use the new identifier.', 13, 10, '$'
msg_other:
        db      'Active BIOS: another BIOS', 13, 10
        db      'Compatible BIOS by Codex marker was not found.', 13, 10, '$'
