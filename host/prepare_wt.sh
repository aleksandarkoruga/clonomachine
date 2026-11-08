#!/usr/bin/env bash
# Wavetables for the synth onto a card (host/prepare_wt.cpp): builds the tool when its sources
# changed, then writes the built-in tables and the given wavetable WAVs (files or folders) as
# <card>/wavetables/NN_<name>.wtl, replacing the .wtl files there. NN is the table's index
# (the synth's aTable / bTable, the first 128); the board loads a table when it is selected.
#
#   host/prepare_wt.sh /run/media/$USER/FPGA                       # the built-ins only
#   host/prepare_wt.sh /run/media/$USER/FPGA ~/wavetables/*.wav    # + these (NNN_name.wtl)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build/host"
BIN="$OUT/prepare_wt"
mkdir -p "$OUT"
if [[ ! -x "$BIN" ]] || [[ -n "$(find "$ROOT/host/prepare_wt.cpp" "$ROOT/model/wt_tables.cpp" "$ROOT/model/wt_tables.h" \
        "$ROOT/model/wav.h" -newer "$BIN")" ]]; then
    g++ -O2 -std=c++17 "$ROOT/host/prepare_wt.cpp" "$ROOT/model/wt_tables.cpp" -o "$BIN"
fi
CARD="${1:?usage: $0 <card mount point> [wavetable WAVs or folders...]}"
shift
[[ -d "$CARD" ]] || { echo "no such folder: $CARD" >&2; exit 1; }
"$BIN" --out "$CARD/wavetables" --builtins --clean "$@"
sync
