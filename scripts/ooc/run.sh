#!/usr/bin/env bash
# Out-of-context synthesis + place-and-route of one unit for its resources and Fmax:
#   scripts/ooc/run.sh <wrapper.v> <tag> <rtl files...>     -> build/ooc/<tag>.{json,log,pnr.log}
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$ROOT/env.sh" >/dev/null
WRAP=$(realpath "$1"); TAG=$2; shift 2
OUT="$ROOT/build/ooc/$TAG"; mkdir -p "$OUT"
FILES=""; for f in "$@"; do FILES="$FILES $(realpath "$f")"; done
cd "$OUT"
cat > ooc.lpf <<'LPF'
LOCATE COMP "clk" SITE "G2";
IOBUF PORT "clk" IO_TYPE=LVCMOS33;
FREQUENCY PORT "clk" 49.152 MHz;
LOCATE COMP "din" SITE "B2";
IOBUF PORT "din" IO_TYPE=LVCMOS33;
LOCATE COMP "dout" SITE "C1";
IOBUF PORT "dout" IO_TYPE=LVCMOS33;
LPF
yosys -q -l synth.log -p "read_verilog -sv $FILES $WRAP; synth_ecp5 -top ooc_top -json ooc.json; tee -o stat.txt stat" >/dev/null
nextpnr-ecp5 --json ooc.json --lpf ooc.lpf --85k --package CABGA381 --speed 6 --timing-allow-fail --seed 1 \
    > pnr.log 2>&1 || true
grep -E "TRELLIS_COMB|TRELLIS_FF|DP16KD|MULT18X18D|TRELLIS_RAMW" pnr.log | sed -n '1,8p'
grep -E "Max frequency for clock" pnr.log | tail -1
