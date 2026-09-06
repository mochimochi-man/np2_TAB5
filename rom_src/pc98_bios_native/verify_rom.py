#!/usr/bin/env python3
import argparse
import hashlib
from pathlib import Path


ROM_SIZE = 0x18000
VENDOR_ID_OFFSET = 0x0B274
MIRROR_ID_OFFSET = 0x17FC0
SIGNATURE = b'Compatible BIOS by Codex'
RESET_OFFSET = 0x17FF0
RESET_VECTOR = bytes((0xEA, 0x00, 0x00, 0x00, 0xF8))


def validate(data: bytes) -> None:
    errors = []
    if len(data) != ROM_SIZE:
        errors.append(f'size is {len(data)}, expected {ROM_SIZE}')
    if len(data) >= VENDOR_ID_OFFSET + len(SIGNATURE):
        if data[VENDOR_ID_OFFSET:VENDOR_ID_OFFSET + len(SIGNATURE)] != SIGNATURE:
            errors.append('vendor identification is missing at physical F3274h')
    if len(data) >= MIRROR_ID_OFFSET + len(SIGNATURE):
        if data[MIRROR_ID_OFFSET:MIRROR_ID_OFFSET + len(SIGNATURE)] != SIGNATURE:
            errors.append('32 KiB identification mirror is missing at physical FFFC0h')
    if len(data) >= RESET_OFFSET + len(RESET_VECTOR):
        if data[RESET_OFFSET:RESET_OFFSET + len(RESET_VECTOR)] != RESET_VECTOR:
            errors.append('reset vector is not JMP F800:0000 at physical FFFF0h')
    if len(data) >= 0x10000:
        expected_low = bytearray((0xFF,)) * 0x10000
        expected_low[VENDOR_ID_OFFSET:VENDOR_ID_OFFSET + len(SIGNATURE)] = SIGNATURE
        if data[:0x10000] != expected_low:
            errors.append('E8000h-F7FFFh contains data outside the F3274h identification')
    if len(data) == ROM_SIZE and sum(data) & 0xFF:
        errors.append('8-bit ROM checksum is not zero')
    if errors:
        raise SystemExit('\n'.join(f'error: {item}' for item in errors))


def main() -> None:
    parser = argparse.ArgumentParser(description='Validate or finalize a PC98N ROM')
    parser.add_argument('input', type=Path)
    parser.add_argument('--fix-checksum', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--variants', action='store_true')
    args = parser.parse_args()

    data = bytearray(args.input.read_bytes())
    if args.fix_checksum:
        if len(data) != ROM_SIZE:
            raise SystemExit(f'error: cannot checksum a {len(data)} byte image')
        data[-1] = (-sum(data[:-1])) & 0xFF

    output = args.output or args.input
    validate(data)
    if args.fix_checksum or args.output:
        output.write_bytes(data)

    digest = hashlib.sha256(data).hexdigest()
    used = sum(value != 0xFF for value in data)
    print(f'OK: {output} ({len(data)} bytes, {used} programmed, sha256 {digest})')

    if args.variants:
        base = output.parent
        f000 = base / 'PC98N_F000.ROM'
        f800 = base / 'PC98N_F800.ROM'
        f000.write_bytes(data[0x8000:])
        f800.write_bytes(data[0x10000:])
        print(f'OK: {f000} ({f000.stat().st_size} bytes, maps at F0000h)')
        print(f'OK: {f800} ({f800.stat().st_size} bytes, maps at F8000h)')


if __name__ == '__main__':
    main()
