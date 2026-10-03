#!/usr/bin/env python3
"""Phase-insensitive comparison of two renderings (e.g. fixed-point vs float phase
vocoder): best lag, waveform SNR, magnitude-spectrogram SNR and log-spectral distance.

    spec_compare.py test.wav ref.wav [more_test.wav ...]
"""
import sys, wave
import numpy as np


def rd(path):
    w = wave.open(path)
    n, ch, sw = w.getnframes(), w.getnchannels(), w.getsampwidth()
    raw = w.readframes(n)
    if sw == 2:
        x = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768
    elif sw == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        x = ((b[:, 0].astype(np.int32) << 8 | b[:, 1].astype(np.int32) << 16 | b[:, 2].astype(np.int32) << 24) >> 8) / 8388608.0
    else:
        x = np.frombuffer(raw, dtype=np.int32).astype(np.float64) / 2 ** 31
    return x.reshape(-1, ch).mean(axis=1)


def stft_mag(x, n=2048, hop=512):
    w = np.hanning(n)
    frames = [np.abs(np.fft.rfft(x[i:i + n] * w)) for i in range(0, len(x) - n, hop)]
    return np.array(frames)


def compare(a, b, maxlag=4096):
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    c = np.fft.irfft(np.fft.rfft(a, 2 * n) * np.conj(np.fft.rfft(b, 2 * n)))
    c = np.concatenate([c[-maxlag:], c[:maxlag + 1]])
    lag = int(np.argmax(c)) - maxlag  # a[i] ~ b[i - lag]
    if lag >= 0:
        a, b = a[lag:], b[:n - lag]
    else:
        a, b = a[:n + lag], b[-lag:]
    snr = 10 * np.log10(np.sum(b ** 2) / (np.sum((a - b) ** 2) + 1e-30))
    A, B = stft_mag(a), stft_mag(b)
    ssnr = 10 * np.log10(np.sum(B ** 2) / (np.sum((A - B) ** 2) + 1e-30))
    eps = 1e-6 * B.max()
    lsd = np.mean(np.sqrt(np.mean((20 * np.log10((A + eps) / (B + eps))) ** 2, axis=1)))
    return lag, snr, ssnr, lsd


if __name__ == "__main__":
    ref = rd(sys.argv[2])
    for t in [sys.argv[1]] + sys.argv[3:]:
        lag, snr, ssnr, lsd = compare(rd(t), ref)
        print("%-40s lag %5d  wave SNR %6.2f dB  spectral SNR %6.2f dB  LSD %5.2f dB" % (t.split("/")[-1], lag, snr, ssnr, lsd))
