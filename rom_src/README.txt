Sources for the two ROMs built into the firmware (main/rom/).

BIOS_Compatible_O.ROM  <- bios_compat_o.asm, via build_compat_o.sh
    A PC-98 compatible BIOS, reconstructed as assembler from a binary whose
    source was not kept. build_compat_o.sh takes that original 32KB image as an
    argument and verifies that -DORIGINAL still reproduces it byte for byte
    before building the 96KB image the emulator loads. Needs nasm.

FONT_ESP.ROM           <- mkfont_esp.py
    A PC-98 FONT.ROM (T98-Next layout) built from the Shinonome bitmap fonts,
    which are public domain. Needs pcf2bdf and the xfonts-shinonome package.

Neither contains any NEC code or data, which is what makes the firmware image
redistributable on its own. See LICENSE.md section 3.
