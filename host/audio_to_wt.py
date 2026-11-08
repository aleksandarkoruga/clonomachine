#!/usr/bin/env python3
"""Make a wavetable from a recording of a pitched sound (a bass one-shot, a held note), like a
synth's pitch-detecting audio import: the pitch is tracked (YIN), single cycles are cut at
evenly spaced points of the sound's voiced part, each cycle is resampled to 2048 samples, made
seamless (crossfaded with the cycle before it), its DC removed and its fundamental turned to
phase 0, so neighbouring frames morph without cancelling. The table is normalised to its peak
and written as a 32-bit float WAV with the 'clm ' chunk that wavetable editors and
model/wt_tables.cpp read.

  .venv/bin/python -I host/audio_to_wt.py [-n frames] [--fmin Hz] [--fmax Hz] [--preview p.wav] <in.wav> <out.wav>

Prints the pitch found and the stretch of the file the frames come from. --preview renders the
table back (scanned over the voiced stretch, at the tracked pitch) to compare with the original.
Basses whose low period is smeared (detuned unison) can lock onto a harmonic: --fmax 130.
"""
import argparse
import struct
import sys

import numpy as np

N0 = 2048


def write_table_wav(path, data, n0=N0):
    """32-bit float WAV with the 'clm ' chunk (frame size), as model/wt_tables.cpp reads it."""
    pcm = np.asarray(data, dtype="<f4").tobytes()
    clm = f"<!>{n0:<4d} 10000000 wavetable".encode()
    clm += b"\0" * (len(clm) & 1)
    fmt = struct.pack("<HHIIHH", 3, 1, 44100, 44100 * 4, 4, 32)
    body = (b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt + b"clm " + struct.pack("<I", len(clm)) + clm
            + b"data" + struct.pack("<I", len(pcm)) + pcm)
    with open(path, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", len(body)) + body)


def read_wav(path):
    """PCM 8/16/24/32 or float 32/64, any channel count -> (mono float64, rate)."""
    b = open(path, "rb").read()
    if b[:4] != b"RIFF" or b[8:12] != b"WAVE":
        raise ValueError(f"{path}: not a WAV file")
    o, fmt, data = 12, None, None
    while o + 8 <= len(b):
        cid, n = b[o:o + 4], struct.unpack_from("<I", b, o + 4)[0]
        if cid == b"fmt ":
            fmt = struct.unpack_from("<HHIIHH", b, o + 8)
            if fmt[0] == 0xFFFE and n >= 26:   # extensible: the sub-format's first two bytes
                fmt = (struct.unpack_from("<H", b, o + 8 + 24)[0],) + fmt[1:]
        elif cid == b"data":
            data = b[o + 8:o + 8 + n]
        o += 8 + n + (n & 1)
    if fmt is None or data is None:
        raise ValueError(f"{path}: no fmt / data chunk")
    tag, ch, sr, _, _, bits = fmt
    if tag == 3:
        x = np.frombuffer(data[:len(data) // (bits // 8) * (bits // 8)], dtype="<f4" if bits == 32 else "<f8")
    elif bits == 8:
        x = (np.frombuffer(data, dtype=np.uint8).astype(np.float64) - 128) / 128
    elif bits == 16:
        x = np.frombuffer(data[:len(data) // 2 * 2], dtype="<i2") / 32768.0
    elif bits == 24:
        a = np.frombuffer(data[:len(data) // 3 * 3], dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
        x = np.where(v >= 1 << 23, v - (1 << 24), v) / float(1 << 23)
    elif bits == 32:
        x = np.frombuffer(data[:len(data) // 4 * 4], dtype="<i4") / float(1 << 31)
    else:
        raise ValueError(f"{path}: {bits}-bit PCM not supported")
    x = np.asarray(x, dtype=np.float64)
    x = x[:len(x) // ch * ch].reshape(-1, ch).mean(axis=1)
    return x, sr


def yin(x, sr, fmin, fmax, hop=512, win=4096, thr=0.15):
    """Per hop: f0 (0 when unvoiced) and the aperiodicity (the YIN dip)."""
    lmin, lmax = int(sr / fmax), min(int(sr / fmin), win // 2)
    f0s, aps = [], []
    for s in range(0, len(x) - win - lmax, hop):
        w = x[s:s + win]
        # difference function via autocorrelation (FFT)
        n = 1 << int(np.ceil(np.log2(2 * win)))
        F = np.fft.rfft(w, n)
        r = np.fft.irfft(F * np.conj(F), n)[:lmax + 1]
        lags = np.arange(lmax + 1)
        e0 = np.concatenate(([0.0], np.cumsum(w * w)))   # e0[n] = sum of the first n squares
        # d(lag) = sum_{i < win - lag} (x_i - x_{i+lag})^2
        d = e0[win - lags] + (e0[win] - e0[lags]) - 2 * r
        d[0] = 0
        cm = np.ones_like(d)
        c = np.cumsum(d[1:])
        cm[1:] = d[1:] * np.arange(1, lmax + 1) / np.maximum(c, 1e-20)
        lag = 0
        for t in range(lmin, lmax):
            if cm[t] < thr:
                while t + 1 < lmax and cm[t + 1] < cm[t]:
                    t += 1
                lag = t
                break
        if lag == 0:
            t = lmin + int(np.argmin(cm[lmin:lmax]))
            lag, ap = t, cm[t]
        else:
            ap = cm[lag]
        if 1 <= lag < lmax:   # parabolic refinement
            a, b_, c_ = cm[lag - 1], cm[lag], cm[lag + 1]
            den = a - 2 * b_ + c_
            lagf = lag + (0.5 * (a - c_) / den if den != 0 else 0)
        else:
            lagf = lag
        f0s.append(sr / lagf if lagf > 0 else 0.0)
        aps.append(ap)
    return np.array(f0s), np.array(aps), hop


def sample_at(x, pos):
    """Windowed-sinc (16 taps, Hann) reads at fractional positions."""
    i = np.floor(pos).astype(np.int64)
    fr = pos - i
    out = np.zeros_like(pos)
    for k in range(-7, 9):
        idx = np.clip(i + k, 0, len(x) - 1)
        t = k - fr
        w = np.sinc(t) * (0.5 + 0.5 * np.cos(np.pi * t / 8.5))
        out += x[idx] * w
    return out


def extract(x, sr, nframes, fmin, fmax):
    f0, ap, hop = yin(x, sr, fmin, fmax)
    rms = np.array([np.sqrt(np.mean(x[i * hop:i * hop + 4096] ** 2) + 1e-30) for i in range(len(f0))])
    voiced = (ap < 0.35) & (rms > rms.max() * 10 ** (-30 / 20)) & (f0 > 0)
    if voiced.sum() < 4:
        raise ValueError("no steady pitch found")
    vi = np.nonzero(voiced)[0]
    a, b = vi[0], vi[-1]
    fv = f0[voiced]
    ref = float(np.median(fv))
    # octave errors: fold the track onto the median's octave
    f0c = f0.copy()
    for i in range(len(f0c)):
        if f0c[i] > 0:
            while f0c[i] > ref * 1.5:
                f0c[i] /= 2
            while f0c[i] < ref / 1.5:
                f0c[i] *= 2
    frames = []
    k = np.arange(N0)
    for j in range(nframes):
        h = a + (b - a) * j / max(nframes - 1, 1)
        lo, hi = max(int(h) - 3, a), min(int(h) + 4, b + 1)
        seg = f0c[lo:hi][voiced[lo:hi]]
        f = float(np.median(seg)) if len(seg) else ref
        P = sr / f
        t = h * hop + 2048        # the YIN window's centre
        t = min(max(t, P + 8), len(x) - P - 16)
        cur = sample_at(x, t + k * P / N0)
        prev = sample_at(x, t + (k - N0) * P / N0)
        w = k / N0
        cyc = cur * (1 - w) + prev * w   # seamless: ends where the next cycle would begin
        C = np.fft.rfft(cyc)
        C[0] = 0
        ph = np.angle(C[1]) + np.pi / 2  # fundamental -> a sine starting at phase 0
        C *= np.exp(-1j * ph * np.arange(len(C)))
        frames.append(np.fft.irfft(C, N0))
    tab = np.concatenate(frames)
    tab /= max(np.max(np.abs(tab)), 1e-12)
    track = (f0c, voiced, hop, a, b)
    return tab * 0.99, ref, a * hop / sr, (b * hop + 4096) / sr, track


def preview(tab, sr, track, nframes):
    """The table played back over the voiced stretch at the tracked pitch (linear reads: the
    frames hold the original's band at that pitch, so nothing folds)."""
    f0c, voiced, hop, a, b = track
    n = int((b - a) * hop + 4096)
    t = np.arange(n) / hop + a
    hops = np.arange(len(f0c))
    fv = np.interp(t, hops[voiced], f0c[voiced])
    ph = np.cumsum(fv / sr) % 1.0
    fpos = np.clip((t - a) / max(b - a, 1) * (nframes - 1), 0, nframes - 1)
    f0i = np.floor(fpos).astype(int)
    f1i = np.minimum(f0i + 1, nframes - 1)
    fw = fpos - f0i
    x = ph * N0
    i0 = np.floor(x).astype(int) % N0
    i1 = (i0 + 1) % N0
    xf = x - np.floor(x)
    T = tab.reshape(nframes, N0)
    r0 = T[f0i, i0] * (1 - xf) + T[f0i, i1] * xf
    r1 = T[f1i, i0] * (1 - xf) + T[f1i, i1] * xf
    return r0 * (1 - fw) + r1 * fw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--frames", type=int, default=64)
    ap.add_argument("--fmin", type=float, default=25.0)
    ap.add_argument("--fmax", type=float, default=1200.0)
    ap.add_argument("--preview", help="render the table back to this WAV")
    ap.add_argument("inp")
    ap.add_argument("out")
    a = ap.parse_args()
    x, sr = read_wav(a.inp)
    tab, f0, t0, t1, track = extract(x, sr, a.frames, a.fmin, a.fmax)
    write_table_wav(a.out, tab.tolist(), N0)
    if a.preview:
        y = preview(tab, sr, track, a.frames) * 0.5
        pcm = (np.clip(y, -1, 1) * 32767).astype("<i2").tobytes()
        fmt = struct.pack("<HHIIHH", 1, 1, sr, sr * 2, 2, 16)
        body = b"WAVE" + b"fmt " + struct.pack("<I", 16) + fmt + b"data" + struct.pack("<I", len(pcm)) + pcm
        with open(a.preview, "wb") as f:
            f.write(b"RIFF" + struct.pack("<I", len(body)) + body)
    print(f"{a.out}: {a.frames} frames, pitch {f0:.2f} Hz, from {t0:.2f} s to {t1:.2f} s of {len(x) / sr:.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
