# Integer prototypes of the two DAC modulators (ported 1:1 to rtl/dsm_dac.v).
import numpy as np
OSR = 256
def interp(x):            # x: s24 per 48 kHz sample -> per modulator tick, linear
    prev = 0
    for v in x:
        d = v - prev
        for k in range(OSR):
            yield prev + ((d * k) >> 8)
        prev = v
def ef4(x):               # 2nd-order error feedback, 4-bit
    e1 = e2 = 0; out = []
    lf = 0xACE1
    for xi in interp(x):
        u = 491520 + ((xi * 3) >> 6)
        lf = ((lf >> 1) ^ (-(lf & 1) & 0xB400)) & 0xFFFF
        w = u - 2 * e1 + e2                      # dither enters the quantizer only,
        q = min(max((w + ((lf & 0x1FFF) - 0x1000) + 32768) >> 16, 0), 15)  # so it is noise-shaped
        e2, e1 = e1, q * 65536 - w
        out.append(q)
    return np.array(out)
def mod2(x):              # 2nd-order CIFB 1-bit (Boser-Wooley, gains 1/2)
    i1 = i2 = 0; out = []; FS = 1 << 23
    for xi in interp(x):
        y = FS if i2 >= 0 else -FS
        i1 += (xi - y) >> 1
        i2 += (i1 - y) >> 1
        out.append(15 if y > 0 else 0)
    return np.array(out)
fs = 48000; n = 2048
f = 997 * n // fs * fs / n  # bin-centred
for name, fn, amp in [("ef4", ef4, 0.9), ("mod2", mod2, 0.5)]:
    t = np.arange(n); x = (amp * 8388607 * np.sin(2*np.pi*f*t/fs)).astype(np.int64)
    o = fn(list(x)).astype(float)
    o = o.reshape(-1, OSR).mean(axis=1)[256:]   # ideal brickwall-ish: boxcar decimate
    o -= o.mean()
    w = np.hanning(len(o)); S = np.abs(np.fft.rfft(o*w))**2
    k = int(round(f*len(o)/fs)); sig = S[k-3:k+4].sum(); noise = S[1:len(S)].sum() - sig
    print(f"{name}: peak code range {fn(list(x[:64])).min()}..{fn(list(x[:64])).max()}  SINAD {10*np.log10(sig/noise):.1f} dB (boxcar decim)")
