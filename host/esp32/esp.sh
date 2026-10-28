#!/usr/bin/env bash
# Build (once) and load the ESP32 pass-through bitstream into the FPGA's SRAM, then run esptool
# through it. The sampler is back after a power cycle (or: host/flash.sh is not needed, just
# reload build/ulx3s_sampler/gateware/radiona_ulx3s.bit with openFPGALoader).
#
#   host/esp32/esp.sh chip_id
#   host/esp32/esp.sh flash_id
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$ROOT/build/esp32_passthru"
source "$ROOT/env.sh" >/dev/null 2>&1 || true
mkdir -p "$OUT"
if [[ ! -f "$OUT/passthru.bit" || "$HERE/passthru.v" -nt "$OUT/passthru.bit" || "$HERE/passthru.lpf" -nt "$OUT/passthru.bit" ]]; then
    yosys -q -p "synth_ecp5 -top esp32_passthru -json $OUT/passthru.json" "$HERE/passthru.v"
    nextpnr-ecp5 -q --85k --package CABGA381 --json "$OUT/passthru.json" --lpf "$HERE/passthru.lpf" \
        --textcfg "$OUT/passthru.config"
    ecppack --compress "$OUT/passthru.config" "$OUT/passthru.bit"
fi
if [[ $# -gt 0 ]]; then
    openFPGALoader -b ulx3s "$OUT/passthru.bit"
    "$ROOT/.venv/bin/python3" "$HERE/esptool_ftdi.py" "$@"
fi
