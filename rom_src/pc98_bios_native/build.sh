#!/bin/sh
# Build the standalone PC98N ROM. Requires only NASM and Python 3.
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

SRC=pc98n.asm
OUT=${1:-PC98N.ROM}
TMP=${OUT}.tmp
trap 'rm -f "$TMP"' EXIT HUP INT TERM

command -v nasm >/dev/null 2>&1 || {
    echo 'nasm was not found in PATH' >&2
    exit 127
}
command -v python3 >/dev/null 2>&1 || {
    echo 'python3 was not found in PATH' >&2
    exit 127
}

nasm -Wall -Werror -f bin -o "$TMP" "$SRC"
python3 verify_rom.py "$TMP" --fix-checksum --output "$OUT" --variants
nasm -Wall -Werror -f bin -o BIOSCHK.COM bioschk.asm
echo 'OK: BIOSCHK.COM (DOS BIOS identification utility)'
