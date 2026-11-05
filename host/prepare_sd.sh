#!/usr/bin/env bash
# Prepare a sample card on the PC (host/prepare_sd.cpp): builds the tool when its sources
# changed, then runs it. With --write the card is backed up first to
# ~/Music/fpga_sd_backup_<date> (skip with --no-backup), because the WAVs are converted in place;
# once it is verified, older backups go to the trash (only the newest is kept).
#
#   host/prepare_sd.sh /run/media/$USER/FPGA                    # report only
#   host/prepare_sd.sh /run/media/$USER/FPGA --write            # backup, convert, write caches
#   host/prepare_sd.sh /run/media/$USER/FPGA --csv table.csv    # + one line per sample
#   host/prepare_sd.sh /run/media/$USER/FPGA --only melody --write   # one folder (and its backup)
#   host/prepare_sd.sh /run/media/$USER/FPGA --target drums=-11      # loudness target (LUFS) of a folder
#   host/prepare_sd.sh /run/media/$USER/FPGA --no-loudness           # no levelling
#
# New samples: copy them into /drums or /melody (subfolders = banks), run with --write. Files
# already converted are left as they are; every cache is rewritten.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/build/host"
BIN="$OUT/prepare_sd"
mkdir -p "$OUT"
if [[ ! -x "$BIN" ]] || [[ -n "$(find "$ROOT/host/prepare_sd.cpp" "$ROOT/fw/transients.c" "$ROOT/fw/transients.h" \
        "$ROOT/fw/wavutil.c" "$ROOT/fw/wavutil.h" "$ROOT/fw/samples.h" "$ROOT/host/loudness.cpp" "$ROOT/host/loudness.h" \
        -newer "$BIN")" ]]; then
    gcc -O2 -c "$ROOT/fw/wavutil.c" -o "$OUT/wavutil.o"
    gcc -O2 -I"$ROOT/fw" -c "$ROOT/fw/transients.c" -o "$OUT/transients.o"
    g++ -O2 -std=c++17 -I"$ROOT/fw" -I"$ROOT/include" "$ROOT/host/prepare_sd.cpp" "$ROOT/host/loudness.cpp" \
        "$OUT/wavutil.o" "$OUT/transients.o" -pthread -o "$BIN"
fi
CARD="${1:?usage: $0 <card mount point> [--write] [--no-backup] [--csv file] [--only folder]}"
shift
ARGS=() WRITE=0 BACKUP=1 ONLY=
while (( $# )); do
    case "$1" in
    --write) WRITE=1; ARGS+=("$1") ;;
    --no-backup) BACKUP=0 ;;
    --only) ONLY="${2:?--only needs a folder}"; ARGS+=("$1" "$2"); shift ;;
    *) ARGS+=("$1") ;;
    esac
    shift
done
if (( WRITE && BACKUP )); then
    SRC="$CARD${ONLY:+/$ONLY}"
    DEST=~/Music/fpga_sd_backup_$(date +%F_%H%M)${ONLY:+_$ONLY}
    echo "backing up $SRC to $DEST ..."
    cp -a "$SRC/." "$DEST"
    if ! diff -rq "$SRC" "$DEST" >/dev/null; then
        echo "backup differs from the card: nothing written"
        exit 1
    fi
    echo "backup verified"
    for old in ~/Music/fpga_sd_backup_*; do
        [[ "$old" == "$DEST" || ! -d "$old" ]] && continue
        echo "older backup $old -> trash"
        gio trash "$old"
    done
fi
"$BIN" "$CARD" "${ARGS[@]}"
if (( WRITE )); then
    sync
    echo "done: unmount the card before taking it out"
fi
