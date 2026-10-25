#!/usr/bin/env bash
# Prepare a sample card on the PC (host/prepare_sd.cpp): builds the tool when its sources
# changed, then runs it. With --write the card is backed up first to
# ~/Music/fpga_sd_backup_<date> (skip with --no-backup), because the WAVs are converted in place.
#
#   host/prepare_sd.sh /run/media/$USER/FPGA                    # report only
#   host/prepare_sd.sh /run/media/$USER/FPGA --write            # backup, convert, write caches
#   host/prepare_sd.sh /run/media/$USER/FPGA --csv table.csv    # + one line per sample
#
# New samples: copy them into /drums or /melody (subfolders = banks), run with --write. Files
# already converted are left as they are; every cache is rewritten.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build/host"
BIN="$OUT/prepare_sd"
mkdir -p "$OUT"
if [[ ! -x "$BIN" ]] || [[ -n "$(find "$ROOT/host/prepare_sd.cpp" "$ROOT/fw/transients.c" "$ROOT/fw/transients.h" \
        "$ROOT/fw/wavutil.c" "$ROOT/fw/wavutil.h" "$ROOT/fw/samples.h" -newer "$BIN")" ]]; then
    gcc -O2 -c "$ROOT/fw/wavutil.c" -o "$OUT/wavutil.o"
    gcc -O2 -I"$ROOT/fw" -c "$ROOT/fw/transients.c" -o "$OUT/transients.o"
    g++ -O2 -std=c++17 -I"$ROOT/fw" -I"$ROOT/include" "$ROOT/host/prepare_sd.cpp" \
        "$OUT/wavutil.o" "$OUT/transients.o" -o "$BIN"
fi
CARD="${1:?usage: $0 <card mount point> [--write] [--no-backup] [--csv file]}"
shift
ARGS=() WRITE=0 BACKUP=1
for a in "$@"; do
    case "$a" in
    --write) WRITE=1; ARGS+=("$a") ;;
    --no-backup) BACKUP=0 ;;
    *) ARGS+=("$a") ;;
    esac
done
if (( WRITE && BACKUP )); then
    DEST=~/Music/fpga_sd_backup_$(date +%F_%H%M)
    echo "backing up $CARD to $DEST ..."
    cp -a "$CARD/." "$DEST"
    diff -rq "$CARD" "$DEST" >/dev/null && echo "backup verified"
fi
"$BIN" "$CARD" "${ARGS[@]}"
if (( WRITE )); then
    sync
    echo "done: unmount the card before taking it out"
fi
