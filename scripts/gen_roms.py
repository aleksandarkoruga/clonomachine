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
