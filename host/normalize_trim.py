#!/usr/bin/env python3
"""Peak-normalize 16-bit WAVs and shorten long silences.

Silence is where the 50 ms RMS (mono mix) stays more than --threshold dB below
the track's peak. Every silent stretch longer than --max-silence is cut down to
exactly --max-silence: half of it is kept after the sound that ends and half
before the sound that starts, joined with a short equal-power crossfade.
Leading silence keeps its last --max-silence, trailing silence its first.

    host/normalize_trim.py DIR_OR_WAVS... [--out DIR] [--peak -1] [--threshold -50]

Output files keep their names and go to --out (default: <input dir>/normalized).
"""
import argparse
import pathlib
import sys
import wave

import numpy as np

HOP = 0.01      # envelope frame, s
SMOOTH = 5      # frames in the RMS window (50 ms)
XFADE = 0.02    # crossfade at each cut, s


def read_wav(path):
    with wave.open(str(path)) as w:
        if w.getsampwidth() != 2:
            sys.exit(f"{path}: only 16-bit PCM is supported")
        sr, ch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    return sr, x.reshape(-1, ch).astype(np.float32) / 32768


def write_wav(path, sr, x):
    # TPDF dither, 1 LSB peak each side
    rng = np.random.default_rng(0)
    d = rng.random(x.shape, dtype=np.float32) - rng.random(x.shape, dtype=np.float32)
    q = np.clip(np.round(x * 32768 + d), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(x.shape[1])
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(q.tobytes())


def silent_runs(sr, x, threshold_db):
    """(start, end) sample ranges where the smoothed RMS is below the threshold."""
    hop = int(sr * HOP)
    n = len(x) // hop
    p = (x[: n * hop] ** 2).mean(axis=1).reshape(n, hop).mean(axis=1)
    p = np.convolve(p, np.ones(SMOOTH) / SMOOTH, mode="same")
    peak = np.abs(x).max()
    silent = p < (peak * 10 ** (threshold_db / 20)) ** 2
    d = np.diff(np.r_[0, silent.astype(np.int8), 0])
    starts, ends = np.where(d == 1)[0] * hop, np.where(d == -1)[0] * hop
    if len(ends) and ends[-1] == n * hop:
        ends[-1] = len(x)       # a trailing run covers the leftover samples too
    return list(zip(starts, ends))


def trim(sr, x, runs, max_silence):
    """Kept sample ranges and the number of silences shortened."""
    keep, half, xf = int(sr * max_silence), int(sr * max_silence / 2), int(sr * XFADE)
    pieces, pos, cuts = [], 0, 0
    for s, e in runs:
        if e - s <= keep:
            continue
        cuts += 1
        if s == 0:
            pos = e - keep
        elif e == len(x):
            pieces.append((pos, s + keep))
            pos = len(x)
        else:
            pieces.append((pos, s + half + xf))     # overlaps the next piece by xf
            pos = e - half
    if pos < len(x):
        pieces.append((pos, len(x)))
    return pieces, cuts


def join(sr, x, pieces):
    xf = int(sr * XFADE)
    t = (np.arange(xf, dtype=np.float32) + 0.5) / xf
    fade_in, fade_out = np.sin(t * np.pi / 2)[:, None], np.cos(t * np.pi / 2)[:, None]
    out = [x[pieces[0][0]:pieces[0][1]].copy()]
    for a, b in pieces[1:]:
        prev, cur = out[-1], x[a:b].copy()
        # only inner cuts make joins, and their first piece runs xf into the cut-out silence
        prev[-xf:] = prev[-xf:] * fade_out + cur[:xf] * fade_in
        out.append(cur[xf:])
    return np.concatenate(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path)
    ap.add_argument("--peak", type=float, default=-1.0, help="target peak, dBFS (default -1)")
    ap.add_argument("--threshold", type=float, default=-50.0,
                    help="silence threshold, dB below the track peak (default -50)")
    ap.add_argument("--max-silence", type=float, default=1.0, help="longest silence kept, s (default 1)")
    args = ap.parse_args()

    files = []
    for p in args.inputs:
        files += sorted(p.glob("*.wav")) if p.is_dir() else [p]
    if not files:
        sys.exit("no WAV files")
    out = args.out or files[0].parent / "normalized"
    out.mkdir(parents=True, exist_ok=True)

    for f in files:
        sr, x = read_wav(f)
        pieces, cuts = trim(sr, x, silent_runs(sr, x, args.threshold), args.max_silence)
        y = join(sr, x, pieces)
        gain = 10 ** (args.peak / 20) / np.abs(y).max()
        write_wav(out / f.name, sr, y * gain)
        print(f"{f.name}: {len(x) / sr:.1f}s -> {len(y) / sr:.1f}s, "
              f"{cuts} silences shortened, gain {20 * np.log10(gain):+.1f} dB")


if __name__ == "__main__":
    main()
