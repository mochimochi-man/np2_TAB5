Sources for the two ROMs built into the firmware (main/rom/).

PC98N.ROM              <- pc98_bios_native/pc98n.asm, via its build.sh
    The PC-98 compatible BIOS, and the one that does the work: 8,636 bytes of
    code at F8000h that bring a machine up to the DOS prompt from a floppy and
    from a hard disk on its own. Needs nasm and python3; build.sh writes
    PC98N.ROM, which is main/rom/PC98N.ROM byte for byte (sha256
    d7c588dd31660c69cd0c24959633de10f6b3ddb4a7b100c892d5d9aca6e8ae9c), and
    verify_rom.py checks the size, the signature and the reset vector before it
    will hand one over.

    Checked against a genuine EPSON BIOS.ROM before it was adopted: not one run
    of 16 bytes or longer is common to the two images, the only printable
    strings they share are 'PSQRVW' and '_^ZY[X' (which are what a PUSH and a
    POP of every register look like as text, and unavoidable in any x86 code),
    and it carries no NEC or EPSON copyright notice - only its own,
    "Compatible BIOS by Codex", at the offset a real machine puts its vendor
    string.

FONT_ESP.ROM           <- mkfont_esp.py
    A PC-98 FONT.ROM (T98-Next layout) built from the Shinonome bitmap fonts,
    which are public domain. Needs pcf2bdf and the xfonts-shinonome package.

Neither contains any NEC code or data, which is what makes the firmware image
redistributable on its own. See LICENSE.md section 3.
