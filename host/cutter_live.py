#!/usr/bin/env python3
"""The break cutter on the PC, standing in for the ESP32 (until its firmware exists).

Runs esp/cutter/cutter.c (build/host/libcutter.so) against the board over the USB console
with "esp sim 1": the board sends its 32nd ticks (synced to the Machinedrum's clock), bpm,
start/stop, voice 0's owner and the channel 7 messages exactly as it will to the ESP, and
takes the cuts back with the same rules (voice 0 only while the cutter owns it: drum page
pitch bend below the centre, or "own" here).

  host/cutter_live.py [--preset N] [--own]

Keys (type + Enter):  p N  preset   |  k NAME VALUE  knob (energy density syncopation anchor
build character release memory surprise, 0..1)  |  c 0|1  conductor manual / auto
own 0|1  hand voice 0 to the cutter / back to MIDI  |  s  status  |  q  quit
The bridge (osc_bridge.py) must not run at the same time.
"""
import argparse
import ctypes
import os
import select
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from ftdi_console import open_console  # noqa: E402

LIB = os.path.join(ROOT, "build", "host", "libcutter.so")
KNOBS = ["energy", "density", "syncopation", "anchor", "build", "character", "release", "memory", "surprise"]
EV_HELLO, EV_TICK = 13, 11
END, ESC, ESC_END, ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD


def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def slip(payload):
    raw = payload + bytes([crc8(payload)])
    out = bytearray([END])
    for x in raw:
        out += bytes([ESC, ESC_END]) if x == END else bytes([ESC, ESC_ESC]) if x == ESC else bytes([x])
    out.append(END)
    return bytes(out)


def build_lib():
    src = [os.path.join(HERE, "cutter_lib.c"), os.path.join(ROOT, "esp", "cutter", "cutter.c"),
           os.path.join(ROOT, "esp", "cutter", "cutter.h")]
    if not os.path.exists(LIB) or any(os.path.getmtime(s) > os.path.getmtime(LIB) for s in src):
        os.makedirs(os.path.dirname(LIB), exist_ok=True)
        subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-I" + os.path.join(ROOT, "esp", "cutter"),
                               src[0], src[1], "-lm", "-o", LIB])
    lib = ctypes.CDLL(LIB)
    lib.cutl_new.restype = ctypes.c_void_p
    lib.cutl_rx.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_char_p]
    lib.cutl_pop.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_char_p]
    lib.cutl_preset.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.cutl_preset_name.restype = ctypes.c_char_p
    lib.cutl_knob.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
    lib.cutl_get_knob.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.cutl_get_knob.restype = ctypes.c_float
    lib.cutl_conductor.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.cutl_state.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.cutl_moves.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.cutl_move_name.restype = ctypes.c_char_p
    return lib


class Frames:
    """SLIP frames out of the console stream; the text between them goes to stdout."""

    def __init__(self):
        self.buf, self.in_frame, self.esc, self.text = bytearray(), False, False, bytearray()

    def feed(self, data):
        frames = []
        for x in data:
            if x == END:
                if self.in_frame and self.buf:
                    if len(self.buf) >= 9 and (len(self.buf) - 1) % 8 == 0 and crc8(self.buf[:-1]) == self.buf[-1]:
                        frames.append(bytes(self.buf[:-1]))
                    self.buf, self.in_frame = bytearray(), False
                else:
                    self.buf, self.in_frame = bytearray(), True
                self.esc = False
            elif self.in_frame:
                if self.esc:
                    self.buf.append(END if x == ESC_END else ESC if x == ESC_ESC else x)
                    self.esc = False
                elif x == ESC:
                    self.esc = True
                else:
                    self.buf.append(x)
            else:
                self.text.append(x)
        if self.text:
            sys.stdout.write(self.text.decode(errors="replace"))
            sys.stdout.flush()
            self.text = bytearray()
        return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", type=int, default=1)
    ap.add_argument("--own", action="store_true", help="hand voice 0 to the cutter at start")
    ap.add_argument("--seed", type=int, default=int(time.time()) & 0xFFFFFF)
    ap.add_argument("--time", type=float, default=0, help="stop after this many seconds (0: run)")
    a = ap.parse_args()
    lib = build_lib()
    c = lib.cutl_new(a.seed)
    lib.cutl_preset(c, a.preset)
    f = open_console()
    f.set_latency_timer(1)
    f.write_data(b"\resp sim 1\r" + (b"cutter 1\r" if a.own else b""))
    frames = Frames()
    hello = slip(struct.pack("<BBBBi", EV_HELLO, 0, ord("E"), ord("S"), 1))
    out = ctypes.create_string_buffer(8 * 16)
    st = (ctypes.c_int * 8)()
    t0 = time.monotonic()
    next_hello = 0
    ticks = sent = 0
    print(f"cutter on the PC: preset {lib.cutl_preset_name(a.preset).decode()}, seed {a.seed}")
    try:
        while True:
            now = time.monotonic() - t0
            if a.time and now > a.time:
                break
            if now >= next_hello:
                f.write_data(hello)
                next_hello = now + 0.5
            for fr in frames.feed(f.read_data(4096) or b""):
                for i in range(0, len(fr), 8):
                    if fr[i] == EV_TICK:
                        ticks += 1
                    lib.cutl_rx(c, time.monotonic() - t0, fr[i:i + 8])
            now = time.monotonic() - t0
            while True:
                n = lib.cutl_pop(c, now, out)
                if not n:
                    break
                f.write_data(slip(out.raw[:8 * n]))
                sent += 1
            r, _, _ = select.select([sys.stdin], [], [], 0.001)
            if r:
                cmd = sys.stdin.readline().split()
                if not cmd:
                    continue
                if cmd[0] == "q":
                    break
                if cmd[0] == "p" and len(cmd) > 1:
                    lib.cutl_preset(c, int(cmd[1]))
                    print("preset", lib.cutl_preset_name(int(cmd[1])).decode())
                elif cmd[0] == "k" and len(cmd) > 2 and cmd[1] in KNOBS:
                    lib.cutl_knob(c, KNOBS.index(cmd[1]), float(cmd[2]))
                elif cmd[0] == "c" and len(cmd) > 1:
                    lib.cutl_conductor(c, int(cmd[1]))
                elif cmd[0] == "own" and len(cmd) > 1:
                    f.write_data(b"cutter %d\r" % int(cmd[1]))
                elif cmd[0] == "s":
                    lib.cutl_state(c, st)
                    knobs = " ".join(f"{k} {lib.cutl_get_knob(c, i):.2f}" for i, k in enumerate(KNOBS))
                    print(f"owner {st[0]} playing {st[1]} n16 {st[2]} conductor {st[3]} (state {st[4]}) "
                          f"preset {lib.cutl_preset_name(st[5]).decode()} step {st[6]} | ticks {ticks} frames {sent}")
                    print(knobs)
                    mv = (ctypes.c_int * 16)()
                    n = lib.cutl_moves(c, mv)
                    print("moves:", " ".join(f"{lib.cutl_move_name(m).decode()} {mv[m]}" for m in range(n) if mv[m]))
    finally:
        f.write_data(b"\resp sim 0\r" + (b"cutter 0\r" if a.own else b""))
        time.sleep(0.2)
        frames.feed(f.read_data(4096) or b"")
        print(f"\nticks {ticks}, frames sent {sent}")
        f.close()


if __name__ == "__main__":
    main()
