#!/usr/bin/env python3
"""Hammer the OSC bridge like a busy Monomachine: CC sweeps on several knobs, 16th-note
triggers and occasional program changes. Run while host/osc_bridge.py is running, then
check `status` (frames / bad / rx overflows) and that the shell shows no garbage.

  host/osc_stress.py [--seconds 10] [--cc-rate 800] [--pc-every 3]
"""
import argparse, socket, struct, time, random

def s(x):
    b = x.encode() + b"\0"
    return b + b"\0" * ((4 - len(b) % 4) % 4)

def msg(addr, *args):
    tags = "," + "".join("i" if isinstance(a, int) else "f" for a in args)
    return s(addr) + s(tags) + b"".join(struct.pack(">i" if isinstance(a, int) else ">f", a) for a in args)

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=57130)
ap.add_argument("--seconds", type=float, default=10); ap.add_argument("--cc-rate", type=float, default=800)
ap.add_argument("--bpm", type=float, default=172); ap.add_argument("--pc-every", type=float, default=3.0)
a = ap.parse_args()
u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); dst = (a.host, a.port)
u.sendto(msg("/bpm", float(a.bpm)), dst)
t0 = time.time(); n_cc = n_note = n_pc = 0
next_cc = next_note = t0; next_pc = t0 + a.pc_every
sixteenth = 60.0 / a.bpm / 4
while time.time() - t0 < a.seconds:
    now = time.time()
    if now >= next_cc:
        ch = random.choice((9, 10)); cc = random.choice((2, 7, 12, 13, 16, 18))
        val = int(63 + 63 * __import__("math").sin(now * 3 + cc))
        u.sendto(msg("/midi/cc", ch, cc, val), dst); n_cc += 1; next_cc += 1.0 / a.cc_rate
    if now >= next_note:
        u.sendto(msg("/midi/note", 9, 12 + random.randrange(16), random.choice((1, 1, 2, 4))), dst); n_note += 1
        next_note += sixteenth
    if a.pc_every and now >= next_pc:
        u.sendto(msg("/midi/pc", 9, random.randrange(41)), dst); n_pc += 1; next_pc += a.pc_every
    time.sleep(0.0002)
print(f"sent {n_cc} CC, {n_note} notes, {n_pc} program changes in {a.seconds:.0f} s")
