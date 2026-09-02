#!/usr/bin/env bash
# Assemble the OPEN-COMPAT PC98 BIOS.
#
# Two things happen here that nasm cannot do on its own:
#
#   - the last byte is a checksum making the whole image sum to a multiple of
#     256, which needs the assembled output to compute;
#   - the reconstruction is verified. This source was recovered from a shipped
#     ROM whose source was not kept, so -DORIGINAL rebuilds that exact 32KB
#     image and compares it against the reference. If that check fails, the
#     reconstruction has drifted and the shipped build cannot be trusted
#     either.
#
# The normal output is a 96KB image (E8000h-FFFFFh) with a corrected reset
# vector, and is deliberately NOT identical to the 32KB reference. np2kai only
# accepts 0x18000 bytes; the original was rejected on size and never ran.
#
# Usage: build_compat_o.sh [reference32k.ROM]
set -e
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SRC=bios_compat_o.asm
OUT=BIOS_Compatible_O.ROM
REF=${1:-}

checksum() {
    python3 - "$1" "$2" <<'PY'
import sys
p, want = sys.argv[1], int(sys.argv[2])
d = bytearray(open(p, 'rb').read())
if len(d) != want:
    raise SystemExit('  size is %d bytes, expected %d' % (len(d), want))
d[-1] = (-sum(d[:-1])) & 0xFF
open(p, 'wb').write(d)
print('  %d bytes, checksum byte 0x%02x, sum %% 256 = %d'
      % (len(d), d[-1], sum(d) % 256))
PY
}

if [ -n "$REF" ]; then
    echo "verifying the reconstruction against $REF"
    nasm -f bin -DORIGINAL -o .orig.tmp "$SRC"
    checksum .orig.tmp 32768
    if cmp -s .orig.tmp "$REF"; then
        echo "  identical - the reconstruction is faithful"
    else
        echo "  DIFFERS from the reference:" >&2
        cmp -l .orig.tmp "$REF" | head -20 >&2
        rm -f .orig.tmp
        exit 1
    fi
    rm -f .orig.tmp
fi

echo "building $OUT (96KB window, reset vector fixed)"
nasm -f bin -o "$OUT.tmp" "$SRC"
checksum "$OUT.tmp" 98304
mv "$OUT.tmp" "$OUT"
ls -l "$OUT"
