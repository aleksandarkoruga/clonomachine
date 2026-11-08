#!/usr/bin/env python3
"""Generate the lookup ROMs shared by the RTL and the C++ golden model.

hann.hex : 1025 x 17 bit, round(65536 * (0.5 - 0.5*cos(2*pi*i/1024)))   (u1.16)
exp2.hex : 257  x 18 bit, round(65536 * 2**(i/256))                      (u2.16)
Both files are $readmemh format, one value per line.
"""
import math, os, sys

out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "rtl")
os.makedirs(out, exist_ok=True)

with open(os.path.join(out, "hann.hex"), "w") as f:
    for i in range(1025):
        f.write("%05x\n" % int(round(65536 * (0.5 - 0.5 * math.cos(2 * math.pi * i / 1024)))))

with open(os.path.join(out, "exp2.hex"), "w") as f:
    for i in range(257):
        f.write("%05x\n" % int(round(65536 * 2 ** (i / 256))))

# ---------------- phase vocoder ROMs (model/pv_model.cpp, rtl/pv_*.v) ----------------
# Constants shared with model/pv_model.h (PV_*). Change both together.
PV_H = 512                 # synthesis hop
PV_N = 4096                # transform length (real), 2048-point complex FFT
PV_CORDIC_ITER = 18        # CORDIC iterations (vectoring and rotation)
PV_INV_SCALE = 0           # inverse FFT stages with /2 scaling
PV_FADE = 64               # reset: old content fade-out / new content fade-in (samples)
PV_RHO0 = 256              # reset: grains R1/R2 enter from u = PV_RHO0 .. PV_H

# pv_sin.hex : 1024 x 17 bit, round(65536 * sin(2*pi*i/4096)), i = 0..1023 (twiddles,
#              analysis window wa[j] = (65536 + cos(2*pi*j/4096)) >> 1); sin(pi/2) = 65536
#              is implied (one 1K x 18 block RAM)
with open(os.path.join(out, "pv_sin.hex"), "w") as f:
    for i in range(PV_N // 4):
        f.write("%05x\n" % int(round(65536 * math.sin(2 * math.pi * i / PV_N))))

# pv_atan.hex : CORDIC angles round(atan(2^-i) / (2*pi) * 2^24), 2^24 = one revolution
with open(os.path.join(out, "pv_atan.hex"), "w") as f:
    for i in range(PV_CORDIC_ITER):
        f.write("%06x\n" % int(round(math.atan(2.0 ** -i) / (2 * math.pi) * (1 << 24))))

# pv_win.hex : synthesis weights, u0.18 (value * 2^18), gain folded in:
#   G = 2*gain*2^PV_INV_SCALE / K^2 * PV_GAIN, gain = 3pi/(3pi+8) (Bungee), K = CORDIC gain,
#   PV_GAIN = 0.5: the grain engine's nominal gain (Hann grains normalised by 1/overlap),
#   so that switching algorithms keeps the level
#   [0    .. 1024]  ws(t) * G, t = 0..1024        (zero phase, symmetric)
#   [1025 .. 1536]  W0(u) * G, u = 0..511          (reset grain R0, one sided)
#   [1537 .. 2048]  W1(u) * G                      (R1, ramped in)
#   [2049 .. 2560]  W2(u) * G                      (R2, ramped in)
#   [2561 .. 2624]  fade-out of old content, u0.16 (65536 = 1), u = 0..PV_FADE-1
K = 1.0
for i in range(PV_CORDIC_ITER):
    K *= math.sqrt(1 + 2.0 ** (-2 * i))
gain = 3 * math.pi / (3 * math.pi + 8)
PV_GAIN = 0.5
G = 2 * gain * 2 ** PV_INV_SCALE / (K * K) * PV_GAIN
def ws(t):
    return 0.25 * (1 + math.cos(2 * math.pi * t / (4 * PV_H))) if abs(t) <= 2 * PV_H else 0.0
def rho(u):
    if u < PV_RHO0:
        return 0.0
    return 0.5 * (1 - math.cos(math.pi * (u - PV_RHO0) / (PV_H - PV_RHO0)))
def fin(u):
    return math.sin(0.5 * math.pi * u / PV_FADE) ** 2 if u < PV_FADE else 1.0
H = PV_H
wvals = [ws(t) for t in range(2 * H + 1)]
wvals += [fin(u) * (ws(u) + ws(u + H) + (1 - rho(u)) * (ws(u - H) + ws(u - 2 * H))) for u in range(H)]
wvals += [rho(u) * ws(u - H) for u in range(H)]
wvals += [rho(u) * ws(u - 2 * H) for u in range(H)]
with open(os.path.join(out, "pv_win.hex"), "w") as f:
    for v in wvals:
        q = int(round(v * G * (1 << 18)))
        assert 0 <= q < (1 << 17), q
        f.write("%05x\n" % q)
    for u in range(PV_FADE):
        f.write("%05x\n" % int(round(65536 * (1 - fin(u)))))

# ---------------- plate reverb taps (model/reverb_model.cpp tap_table, rtl/reverb_unit.v) ----------------
# rv_t0.hex / rv_t1.hex : 1024 x 18 bit two's complement, Q15 4-point Lagrange taps t0 / t1 at the
# fraction k/1024 (t2, t3 = t1, t0 at 1024-k). Python's >> floors like the model's.
def rv_tap(which, k):
    f = k << 5
    fm1, fm2 = f - 32768, f - 65536
    if which == 0:
        a = (f * fm1) >> 15
        return (((a * fm2) >> 15) * -21845) >> 17
    c = ((f + 32768) * fm2) >> 15
    return ((c * fm1) >> 15) >> 1
for which in (0, 1):
    with open(os.path.join(out, "rv_t%d.hex" % which), "w") as f:
        for k in range(1024):
            v = rv_tap(which, k)
            assert -(1 << 17) <= v < (1 << 17), v
            f.write("%05x\n" % (v & 0x3FFFF))

# ---------------- wavetable synth (rtl/synth_osc.v, model/synth_osc.h) ----------------
# sy_sinc.hex : 2048 x 18 bit two's complement, the 8-tap Kaiser-windowed sinc of
#               model/synth_osc.h (fc 0.92, beta 7) at phases p = 0..256 (entry p*8 + j, Q1.17,
#               each phase normalised to unity DC gain); 257*8 = 2056 > 2048, so phase 256 (= phase
#               0 shifted by one sample) is formed in the RTL from phase 0
def i0(x):
    s, t = 1.0, 1.0
    for k in range(1, 30):
        t *= x * x / (4.0 * k * k)
        s += t
    return s
SY_T, SY_P = 8, 256
with open(os.path.join(out, "sy_sinc.hex"), "w") as f:
    for p in range(SY_P):
        c = []
        for j in range(SY_T):
            x = (j - (SY_T // 2 - 1)) - p / SY_P
            sx = 0.92 if x == 0 else math.sin(math.pi * 0.92 * x) / (math.pi * x)
            r = x / (SY_T / 2)
            w = 0.0 if abs(r) >= 1 else i0(7.0 * math.sqrt(1 - r * r)) / i0(7.0)
            c.append(sx * w)
        s = sum(c)
        for v in c:
            q = int(round(v / s * 131072))
            assert -(1 << 17) <= q < (1 << 17), q
            f.write("%05x\n" % (q & 0x3FFFF))

# sy_fn.hex : 1024 x 18 bit, four 256-entry tables (linear interpolation in the RTL; the entry
#             after 255 is the constant at the region's end):
#   [0..255]    log2(1 + i/256)            Q0.17   (end: 1.0)
#   [256..511]  2^(i/256)                  Q1.16   (end: 2.0)
#   [512..767]  sin(pi/2 * i/256)          Q0.17   (end: 1.0, saturated to 131071)
#   [768..1023] tanh(i/32), x in [0, 8)    Q0.17   (end: tanh 8)
with open(os.path.join(out, "sy_fn.hex"), "w") as f:
    for i in range(256):
        f.write("%05x\n" % min(131071, int(round(math.log2(1 + i / 256) * 131072))))
    for i in range(256):
        f.write("%05x\n" % int(round(2 ** (i / 256) * 65536)))
    for i in range(256):
        f.write("%05x\n" % min(131071, int(round(math.sin(math.pi / 2 * i / 256) * 131072))))
    for i in range(256):
        f.write("%05x\n" % min(131071, int(round(math.tanh(i / 32) * 131072))))

# sy_ffn.hex : 1024 x 18 bit, the filter engine's function unit (rtl/synth_filt.v), linear
#              interpolation between entries i and i+1 of a region:
#   [0..256]    tanh(i/32), x in [0, 8]           Q0.17 (saturated to 131071)
#   [512..768]  1/(1 + i/256)                     Q0.17 (entry 0 = 1.0 saturated to 131071)
with open(os.path.join(out, "sy_ffn.hex"), "w") as f:
    vals = [0] * 1024
    for i in range(257):
        vals[i] = min(131071, int(round(math.tanh(i / 32) * 131072)))
        vals[512 + i] = min(131071, int(round(131072 / (1 + i / 256))))
    for v in vals:
        f.write("%05x\n" % v)

# sy_hb.hex : 19 x 18 bit, the synth's 2x -> 1x half-band decimator (model/synth_dsp.h Halfband(71)):
#             entry 0 = the centre tap (0.5), entry k = the taps at +-(2k - 1) from it, Q1.17
def i0b(x):
    s, t = 1.0, 1.0
    for k in range(1, 40):
        t *= x * x / (4.0 * k * k)
        s += t
    return s
HB_LEN, HB_BETA = 71, 8.6
hb = []
M = HB_LEN - 1
for i in range(HB_LEN):
    x = i - M / 2
    sv = 0.5 if x == 0 else math.sin(math.pi * 0.5 * x) / (math.pi * x)
    r = 2.0 * i / M - 1
    hb.append(sv * i0b(HB_BETA * math.sqrt(max(0.0, 1 - r * r))) / i0b(HB_BETA))
hs = sum(hb)
hb = [v / hs for v in hb]
with open(os.path.join(out, "sy_hb.hex"), "w") as f:
    f.write("%05x\n" % (int(round(hb[M // 2] * 131072)) & 0x3FFFF))
    for k in range(1, 19):
        f.write("%05x\n" % (int(round(hb[M // 2 + 2 * k - 1] * 131072)) & 0x3FFFF))
