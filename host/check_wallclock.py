#!/usr/bin/env python3
"""Board check of bgAdvance: a note starts from pos + the file's own playhead in real time
(the time since Start at the file's sample rate, wrapped at its length), whatever the tempo
and rate, also after a buffer switch. Voice 1 at rate 0 (the playhead stays on the latch)
plays note 0 three times: the second start must have moved by the time in between at the
file's speed, the third (after a switch to the next buffer and back) by the time since the
first. Restores voice 1's rate, buffer and bgAdvance afterwards. Nothing else may hold the
console (midimap_web.py, osc_bridge.py ...)."""
import os, re, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ftdi_console import open_console

FS = 48014.3
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
SLOT1 = re.compile(r"slot 1: lib \d bank (\d+) buf (\d+)  phasor (\d+).*?ratio ([0-9.]+)  rate (-?[0-9.]+)"
                   r".*?bgAdvance (\d)")
def note():  # the latch, read right after the note (rate 0: the playhead stays there)
    st = cmd("note 1 0 1\rstatus", 0.6)
    m = SLOT1.search(st)
    return int(re.search(r"samples (\d+)", st).group(1)), int(m.group(3)), (int(m.group(1)), int(m.group(2)))

rd(0.3)
m = SLOT1.search(cmd("status", 0.5))
bank, buf, ratio, rate0, bg0 = int(m.group(1)), int(m.group(2)), float(m.group(4)), m.group(5), m.group(6)
lst = cmd("list 1 %d" % bank, 1.0)
m = re.search(r"^\s*%d \*? ?.*?(\d+) ms (\d+) Hz" % buf, lst, re.M)
ms, sr = int(m.group(1)), int(m.group(2))
frames = ms * sr / 1000.0          # to the ms: only matters where the difference wraps
srf = sr / FS
nbuf = len(re.findall(r"^\s*\d+ [* ] ", lst, re.M))

cmd("set 1 rate 0"); cmd("set 1 bgAdvance 1")
s1, p1, b1 = note()
time.sleep(1.5)
s2, p2, b2 = note()
if nbuf > 1:
    cmd("set 1 bufNum %d" % ((buf + 1) % nbuf), 0.6)
    cmd("set 1 bufNum %d" % buf, 0.6)
time.sleep(0.5)
s3, p3, b3 = note()
cmd("set 1 rate %s" % rate0); cmd("set 1 bgAdvance %s" % bg0); cmd("set 1 gateOff 1")
f.close()

def err(moved, want):
    d = (moved - want) % frames
    return min(d, frames - d)
print("file %d/%d: %d ms at %d Hz (%.0f frames), tempo ratio %.3f (must not matter)" % (bank, buf, ms, sr, frames, ratio))
if not (b1 == b2 == b3 == (bank, buf)):
    print("the buffer changed under the test (%s %s %s): run it again" % (b1, b2, b3))
for name, (sa, pa), (sb, pb) in (("second note", (s1, p1), (s2, p2)),
                                 ("after the switch" if nbuf > 1 else "third note", (s1, p1), (s3, p3))):
    want = (sb - sa) * srf
    moved = (pb - pa) % frames
    e = err(moved, want)
    print("%-17s start moved %8.0f frames, real time at the file's speed %8.0f (off by %4.0f)%s"
          % (name, moved, want % frames, e, "" if e < 0.004 * frames + 100 else "  <-- WRONG"))
    print("%-17s (with the tempo it would be %8.0f)" % ("", (want * ratio) % frames))
