#!/usr/bin/env bash
# Build our MicroPython for the on-board ESP32: board esp/boards/ULX3S_CUTTER, C module
# esp/mpy (the break cutter + its FPGA link), frozen app esp/py. Output in build/esp32/.
#   host/esp32/build.sh [clean]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$ROOT/host/esp32/env.sh"
make -C "$MPY_DIR/ports/esp32" BOARD_DIR="$ROOT/esp/boards/ULX3S_CUTTER" BUILD="$ROOT/build/esp32" \
    USER_C_MODULES="$ROOT/esp/mpy/micropython.cmake" -j"$(nproc)" "$@"
