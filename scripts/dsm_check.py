#!/usr/bin/env python3
"""In-band SINAD of a dsm_dac code dump (sim/tb_dsm.cpp): dsm_check.py file.bin"""
import sys, numpy as np
o = np.fromfile(sys.argv[1], dtype=np.uint8).astype(float)
fs_mod = 48000 * 256
o -= o.mean()
S = np.abs(np.fft.rfft(o * np.hanning(len(o)))) ** 2
fr = np.arange(len(S)) * fs_mod / len(o)
f = 31 * 48000 / 1024
k = int(np.argmin(abs(fr - f)))
band = (fr > 20) & (fr < 20000)
sig = S[k - 3:k + 4].sum()
noise = S[band].sum() - sig
print(f"codes {int(o.min()+np.fromfile(sys.argv[1],dtype=np.uint8).mean())}.. in-band SINAD {10*np.log10(sig/noise):.1f} dB")
