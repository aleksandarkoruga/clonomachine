#!/usr/bin/env bash
# Write the sampler to the ULX3S SPI flash so it boots standalone at power-up:
#   offset 0     : bitstream
#   offset 4 MB  : firmware boot image (length, crc32, sampler.bin) read by the BIOS
# Usage: host/flash.sh [all|fw|bit]      (default: all)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$ROOT/env.sh"
BIT="$ROOT/build/ulx3s_sampler/gateware/radiona_ulx3s.bit"
FW="$ROOT/fw/sampler.bin"
FBI="$ROOT/build/sampler.fbi"
FW_OFFSET=0x400000   # keep in sync with FW_FLASH_OFFSET in soc/ulx3s_sampler.py
what="${1:-all}"

if [[ "$what" == all || "$what" == bit ]]; then
    openFPGALoader -b ulx3s -f --unprotect-flash --verify "$BIT"
fi
if [[ "$what" == all || "$what" == fw ]]; then
    python3 "$ROOT/deps/litex/litex/soc/software/crcfbigen.py" "$FW" -f -l -o "$FBI"
    openFPGALoader -b ulx3s -f --unprotect-flash --verify -o "$FW_OFFSET" --file-type bin "$FBI"
fi
echo "flash written ($what). Power-cycle or press PWR to boot from flash."
