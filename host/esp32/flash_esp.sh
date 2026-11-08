#!/usr/bin/env bash
# Flash our MicroPython (build/esp32, host/esp32/build.sh) to the on-board ESP32 through the
# pass-through bitstream, then the FPGA boots from its flash again (gateware + firmware there).
#   host/esp32/flash_esp.sh [--erase]     --erase: wipe the whole ESP flash first (new layout)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$ROOT/env.sh" >/dev/null 2>&1 || true
ERASE=()
[[ "${1:-}" == "--erase" ]] && ERASE=(--erase-all)
"$ROOT/host/esp32/esp.sh"   # builds the pass-through if needed
openFPGALoader -b ulx3s "$ROOT/build/esp32_passthru/passthru.bit"
(cd "$ROOT/build/esp32" && "$ROOT/.venv/bin/python3" "$ROOT/host/esp32/esptool_ftdi.py" --chip esp32 -b 460800 \
    --before default_reset --after hard_reset write_flash "${ERASE[@]}" "@flash_args")
openFPGALoader -b ulx3s --reset
echo "ESP flashed, the FPGA booted from its flash again"
