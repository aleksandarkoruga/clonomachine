#!/usr/bin/env bash
# Place-and-route a built gateware with several nextpnr seeds in parallel and print each
# seed's Fmax per clock and the start of its worst audio path.
#
#   scripts/seed_sweep.sh <gateware dir> <tag> [seeds...]      (default seeds 1..8)
#
# Logs go to build/seeds/<tag>_s<seed>.log. The best seed goes into nextpnr_seed in
# soc/ulx3s_sampler.py; it only holds for the netlist it was found on.
set -u
GW=$1; TAG=$2; shift 2
SEEDS=${*:-1 2 3 4 5 6 7 8}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
. "$ROOT/env.sh"
OUT=$ROOT/build/seeds; mkdir -p "$OUT"
for s in $SEEDS; do
    nextpnr-ecp5 --json "$GW/radiona_ulx3s.json" --lpf "$GW/radiona_ulx3s.lpf" --85k --package CABGA381 \
        --speed 6 --timing-allow-fail --seed "$s" > "$OUT/${TAG}_s$s.log" 2>&1 &
done
wait
for s in $SEEDS; do
    python3 - "$OUT/${TAG}_s$s.log" <<'EOF'
import re, sys
t = open(sys.argv[1]).read()
f = {c: (re.findall(r"Max frequency for clock +'\$glbnet\$%s_clk': ([\d.]+)" % c, t) or ['-'])[-1]
     for c in ('usb', 'audio', 'sys')}
m = re.search(r"Critical path report for clock '\$glbnet\$audio_clk'.*?Source (\S+)", t, re.S)
print("%-14s usb %6s  audio %6s  sys %6s  : %s" % (sys.argv[1].split('/')[-1][:-4], f['usb'], f['audio'],
      f['sys'], m.group(1)[:60] if m else ''))
EOF
done
