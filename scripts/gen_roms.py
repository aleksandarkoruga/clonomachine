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
