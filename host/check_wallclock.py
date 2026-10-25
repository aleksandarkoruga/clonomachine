#!/usr/bin/env python3
"""Board check of the bgAdvance wall clock (BG_INC): a melody voice at rate 0.25 plays a
note, then the same note again a few seconds later. The second latch must move by the wall
clock (tempo only), while the playhead in between runs at rate 0.25. Prints both, unwrapped
by the loop length. Changes voice 1's settings and turns clock follow off (power-cycle or
'clock 1' afterwards). The bridge must not be running."""
import os, re, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ftdi_console import open_console

f = open_console()
def rd(t):
    out, end = b"", time.time() + t
    while time.time() < end:
        d = f.read_data(4096)
        if d: out += d
        else: time.sleep(0.003)
    return out.decode(errors="replace").replace("\r", "")
def cmd(c, t=0.4):
    f.write_data(c.encode() + b"\r")
    return rd(t)
def snap():
    st = cmd("status", 0.5)
    smp = int(re.search(r"samples (\d+)", st).group(1))
    m = re.search(r"slot 1: lib \d bank (\d+) buf (\d+)  phasor (\d+).*?ratio ([0-9.]+)", st)
    return smp, int(m.group(3)), float(m.group(4)), int(m.group(1)), int(m.group(2))

rd(0.3)
cmd("clock 0"); cmd("bpm 120"); cmd("set 1 envHold 1"); cmd("set 1 rate 0.25")
cmd("note 1 12 1"); s1, p1, ratio, bank, buf = snap()
time.sleep(2.0)
s2a, p2a, _, _, _ = snap()
cmd("note 1 12 1"); s2, p2, _, _, _ = snap()
lst = cmd("list 1 %d" % bank, 1.0)
m = re.search(r"^\s*%d \*? ?.*?(\d+) ms (\d+) Hz" % buf, lst, re.M)
ms, sr = int(m.group(1)), int(m.group(2))
frames = round(ms * sr / 1000 / 64) * 64 if ms % 1000 else ms * sr // 1000
srf = sr / 48014.3
wall = (s2 - s1) * ratio * srf
print("loop ~%d frames at %d Hz, tempo ratio %.3f" % (frames, sr, ratio))
print("latch moved   %7d frames, wall clock predicts %7.0f" % ((p2 - p1) % frames, wall))
print("playhead ran  %7d frames between the first two reads (rate 0.25)" % ((p2a - p1) % frames))
cmd("set 1 rate 1"); cmd("set 1 envHold 0"); cmd("set 1 gateOff 1"); cmd("clock 1")
f.close()
