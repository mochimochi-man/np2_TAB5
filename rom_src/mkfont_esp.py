#!/usr/bin/env python3
"""Build a compatible PC-98 FONT.ROM (T98-Next layout) from the Shinonome fonts.

Shinonome is public domain, so the result can be redistributed freely. Nothing
is taken from NEC's ROM.

Layout, read out of np2kai's font/fontv98.c (V98FILESIZE = 0x46800 = 288768):

    0x0000  8x8  ANK, 256 chars, 8 bytes each
    0x0800  8x16 ANK 0x00-0x7F, 16 bytes each
    0x1000  8x16 ANK 0x80-0xFF, 16 bytes each
    0x1800  16x16 kanji, one JIS ku (row) after another starting at ku 1.
            Each ku is 0x60 chars * 32 bytes; each char is 16 bytes of the
            left half followed by 16 bytes of the right half.

Usage: mkfont98.py OUT.ROM
"""

import gzip
import subprocess
import sys
import os

ROM_SIZE   = 0x46800
KU_BYTES   = 0x60 * 32          # 96 chars per ku, 32 bytes per char
KANJI_BASE = 0x1800

# The "r" face is the JIS X 0201 one - ASCII in 0x20-0x7E (with the yen sign at
# 0x5C) and half-width katakana in 0xA1-0xDF, which is exactly the PC-98 ANK
# set. The "a" face next to it is ISO 8859-1 and is NOT used: it was merged in
# here originally, on the assumption that it carried the katakana, and instead
# it overwrote them with Latin-1 - which is why 0xB1 came out as the
# plus-minus sign rather than as katakana A.
#
#   $ pcf2bdf shnm8x16r.pcf | grep CHARSET_REGISTRY   ->  "JISX0201.1976"
#   $ pcf2bdf shnm8x16a.pcf | grep CHARSET_REGISTRY   ->  "ISO8859"
ANK16 = "/usr/share/fonts/X11/misc/shnm8x16r.pcf.gz"
KANJI = "/usr/share/fonts/X11/misc/shnmk16.pcf.gz"


def bdf_from_pcf(path):
    """pcf2bdf needs a plain file, so gunzip into a temp and convert."""
    raw = "/tmp/" + os.path.basename(path)[:-3]
    with gzip.open(path, "rb") as fi, open(raw, "wb") as fo:
        fo.write(fi.read())
    return subprocess.run(["pcf2bdf", raw], capture_output=True, text=True,
                          check=True).stdout


def parse_bdf(text):
    """-> {encoding: (width, height, [row bytes...])}, rows padded to bytes."""
    glyphs = {}
    enc = None
    bbx = None
    bits = None
    for line in text.splitlines():
        if line.startswith("ENCODING "):
            enc = int(line.split()[1])
        elif line.startswith("BBX "):
            bbx = [int(v) for v in line.split()[1:]]
        elif line == "BITMAP":
            bits = []
        elif line == "ENDCHAR":
            if enc is not None and bits is not None:
                glyphs[enc] = (bbx, bits)
            enc = bbx = bits = None
        elif bits is not None:
            bits.append(line.strip())
    return glyphs


def rows_of(glyphs, enc, width, height):
    """Glyph as `height` rows of `width//8` bytes, blank when absent.

    BDF stores each row padded to whole bytes and positions the bitmap by its
    bounding box, so the box offset has to be honoured or accents and descenders
    land on the wrong line."""
    nbytes = (width + 7) // 8
    out = [bytes(nbytes)] * height
    g = glyphs.get(enc)
    if not g:
        return out
    (bw, bh, bx, by), bits = g
    rowbytes = (bw + 7) // 8
    # BDF rows run top-down; the box sits `by` above the baseline, and the
    # baseline here is at height + descent.
    top = height - bh - (by + (height - 16 if height > 16 else 0))
    top = max(0, min(top, height - bh))
    for i, hexrow in enumerate(bits):
        y = top + i
        if not (0 <= y < height):
            continue
        val = int(hexrow, 16) if hexrow else 0
        val <<= (rowbytes * 8 - len(hexrow) * 4)      # left-align in its bytes
        b = val.to_bytes(rowbytes, "big")[:nbytes]
        out[y] = b.ljust(nbytes, b"\0")
    return out


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    rom = bytearray(b"\0" * ROM_SIZE)

    ank = parse_bdf(bdf_from_pcf(ANK16))
    knj = parse_bdf(bdf_from_pcf(KANJI))

    # --- 8x16 ANK, both halves, straight from the JIS X 0201 face. Codes it
    # does not define (0x80-0xA0, 0xE0-0xFF) stay blank rather than being
    # filled from some other character set: a blank cell is obviously missing,
    # a wrong glyph is not.
    for c in range(256):
        rows = rows_of(ank, c, 8, 16)
        off = 0x0800 + c * 16 if c < 0x80 else 0x1000 + (c - 0x80) * 16
        rom[off:off + 16] = b"".join(rows)

    # --- 8x8 ANK: vertically halve the 16-row glyph by OR-ing scanline pairs,
    # which keeps thin strokes visible instead of dropping every other row.
    for c in range(256):
        rows = rows_of(ank, c, 8, 16)
        packed = bytes(rows[i * 2][0] | rows[i * 2 + 1][0] for i in range(8))
        rom[c * 8:c * 8 + 8] = packed

    # --- 16x16 kanji, ku 1..94. Shinonome is indexed by JIS code, so ku/ten
    # convert as enc = (ku + 0x20) << 8 | (ten + 0x20).
    #
    # The slot within a ku block is `ten`, NOT `ten - 1`. np2kai reads a block
    # as 96 slots for j = 0x20..0x7F (fontv98.c, v98knjcpy) and the emulator
    # addresses a glyph as (kc & 0x7f7f) << 4 (maketext.c), which works out to
    # j = ten + 0x20. Slot 0 is therefore ten 0 and goes unused, and writing
    # from slot 0 shifted every character in the row by one - the whole kanji
    # set came out as its own neighbours.
    written = 0
    for ku in range(1, 95):
        base = KANJI_BASE + (ku - 1) * KU_BYTES
        for ten in range(1, 95):
            enc = ((ku + 0x20) << 8) | (ten + 0x20)
            if enc not in knj:
                continue
            rows = rows_of(knj, enc, 16, 16)
            off = base + ten * 32
            if off + 32 > ROM_SIZE:
                continue
            rom[off:off + 16] = bytes(r[0] for r in rows)          # left half
            rom[off + 16:off + 32] = bytes(r[1] for r in rows)     # right half
            written += 1

    with open(sys.argv[1], "wb") as f:
        f.write(rom)
    print(f"{sys.argv[1]}: {len(rom)} bytes, {written} kanji")
    return 0


if __name__ == "__main__":
    sys.exit(main())
