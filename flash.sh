#!/usr/bin/env bash
# Flash the firmware and open the serial monitor.
# Usage: ./flash.sh [PORT]     (default port: /dev/ttyACM0)
#
# The Tab5 appears over its USB-C socket as a USB-Serial-JTAG device: on Linux
# usually /dev/ttyACM0, on Windows a COM port. idf.py takes the flash mode and
# the offsets from sdkconfig, so this writes the bootloader, the partition
# table and the app together and in qio mode - which matters: the app alone
# will run from a dio-mode bootloader, just more slowly.
# Exit the monitor with Ctrl-].
set -e
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-/dev/ttyACM0}"

if ! command -v idf.py >/dev/null 2>&1; then
  for e in "$IDF_PATH/export.sh" "$HOME/esp/esp-idf-v5.5/export.sh" "$HOME/esp/esp-idf/export.sh"; do
    if [ -f "$e" ]; then . "$e" >/dev/null; break; fi
  done
fi
if ! command -v idf.py >/dev/null 2>&1; then
  echo "ERROR: ESP-IDF not found. Run: . \$HOME/esp/esp-idf-v5.5/export.sh" >&2
  exit 1
fi

idf.py -p "$PORT" flash monitor
