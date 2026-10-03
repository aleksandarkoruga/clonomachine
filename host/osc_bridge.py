#!/usr/bin/env python3
"""OSC (UDP) -> SLIP event bridge to the ULX3S sampler over the USB console.

Stand-in for the ESP32-S3 relay: same OSC namespace, same 8-byte events.
SuperCollider (or TouchOSC, ...) sends to this host; the bridge forwards to the board.

  /bpm f                        tempo
  /midi/note  i chan i note i vel      (vel 0 = note off)
  /midi/noteoff i chan i note
  /midi/cc    i chan i cc i val
  /midi/pc    i chan i prog
  /midi/bend  i chan i val14    (0..16383)
  /slot/<n>/<param> [f]         BufGrainPlay names: bufNum pos rate pitch grainDur overlap
                                prob separation randPos randomPitch, plus bank trig envBars
                                envDur envHold gateOff repeats interval amp ... (sampler_events.h)
  /slot/<n>/trig
  /transport/start, /transport/stop, /clock

MIDI channels are SuperCollider's 0-based numbers (slot 0 = chan 9, slot 1 = chan 10).

  host/osc_bridge.py [--osc-port 57130] [--port ftdi://ftdi:0x6015/1 | /dev/ttyUSB0] [-v]

The board's console output is printed; lines typed on stdin go to the board's shell.
"""
import argparse
import os
import re
import select
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
EVENTS_H = os.path.join(HERE, "..", "include", "sampler_events.h")

EV_NOTE_ON, EV_NOTE_OFF, EV_CC, EV_PC, EV_BEND, EV_BPM, EV_CLOCK, EV_START, EV_STOP, EV_RAW_PARAM, EV_LOAD_BUF = range(11)
SLIP_END, SLIP_ESC, SLIP_ESC_END, SLIP_ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD


def load_params(path=EVENTS_H):
    """{oscname_lower: (id, frac_bits)} from the SMP_PARAMS X() list (id = position)."""
    params = {}
    for i, m in enumerate(re.finditer(r"^\s*X\(\s*(\w+),\s*(\w+),\s*(\d+)\)", open(path).read(), re.M)):
        params[m.group(2).lower()] = (i, int(m.group(3)))
    return params


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def event(type_, chan=0, a=0, b=0, value=0):
    return struct.pack("<BBBBi", type_ & 0xFF, chan & 0xFF, a & 0xFF, b & 0xFF, int(value))


def slip_frame(events):
    payload = b"".join(events)
    payload += bytes([crc8(payload)])
    out = bytearray([SLIP_END])
    for c in payload:
        if c == SLIP_END:
            out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif c == SLIP_ESC:
            out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(c)
    out.append(SLIP_END)
    return bytes(out)


# ---- OSC decoding ------------------------------------------------------------------------
def _osc_string(data, i):
    j = data.index(b"\0", i)
    s = data[i:j].decode(errors="replace")
    return s, (j + 4) & ~3


def parse_osc(data):
    """Yields (address, [args]) for a message or (recursively) a bundle."""
    if data.startswith(b"#bundle\0"):
        i = 16
        while i + 4 <= len(data):
            n = struct.unpack(">i", data[i:i + 4])[0]
            yield from parse_osc(data[i + 4:i + 4 + n])
            i += 4 + n
        return
    addr, i = _osc_string(data, 0)
    if i >= len(data) or data[i:i + 1] != b",":
        yield addr, []
        return
    tags, i = _osc_string(data, i)
    args = []
    for t in tags[1:]:
        if t == "i":
            args.append(struct.unpack(">i", data[i:i + 4])[0]); i += 4
        elif t == "f":
            args.append(struct.unpack(">f", data[i:i + 4])[0]); i += 4
        elif t == "d":
            args.append(struct.unpack(">d", data[i:i + 8])[0]); i += 8
        elif t == "h":
            args.append(struct.unpack(">q", data[i:i + 8])[0]); i += 8
        elif t == "s":
            s, i = _osc_string(data, i); args.append(s)
        elif t == "b":
            n = struct.unpack(">i", data[i:i + 4])[0]; args.append(data[i + 4:i + 4 + n]); i = (i + 4 + n + 3) & ~3
        elif t == "T":
            args.append(1)
        elif t in "FN":
            args.append(0)
    yield addr, args


# ---- OSC -> events ---------------------------------------------------------------------------
class Mapper:
    def __init__(self, params):
        self.params = params

    def __call__(self, addr, args):
        n = [int(round(a)) if isinstance(a, (int, float)) else 0 for a in args]
        f = [float(a) if isinstance(a, (int, float)) else 0.0 for a in args]
        if addr == "/bpm" and f:
            return [event(EV_BPM, value=round(f[0] * 1000))]
        if addr == "/midi/note" and len(n) >= 3:
            return [event(EV_NOTE_ON if n[2] > 0 else EV_NOTE_OFF, n[0], n[1], n[2])]
        if addr == "/midi/noteoff" and len(n) >= 2:
            return [event(EV_NOTE_OFF, n[0], n[1])]
        if addr == "/midi/cc" and len(n) >= 3:
            return [event(EV_CC, n[0], n[1], n[2])]
        if addr == "/midi/pc" and len(n) >= 2:
            return [event(EV_PC, n[0], n[1])]
        if addr == "/midi/bend" and len(n) >= 2:
            return [event(EV_BEND, n[0], value=max(0, min(16383, n[1])))]
        if addr == "/transport/start":
            return [event(EV_START)]
        if addr == "/transport/stop":
            return [event(EV_STOP)]
        if addr == "/clock":
            return [event(EV_CLOCK)]
        m = re.match(r"^/slot/(\d+)/(\w+)$", addr)
        if m:
            slot, name = int(m.group(1)), m.group(2).lower()
            if name not in self.params:
                return None
            pid, frac = self.params[name]
            x = f[0] if f else 0.0
            if name == "pos" and x < 0:   # BufGrainPlay: pos < 0 = run free
                return [event(EV_RAW_PARAM, 0, slot, pid, -1)]
            return [event(EV_RAW_PARAM, 0, slot, pid, round(x * (1 << frac)))]
        return None


# ---- link to the board -----------------------------------------------------------------------
class FtdiLink:
    def __init__(self, url, baud):
        sys.path.insert(0, HERE)
        from ftdi_console import open_console
        self.f = open_console(baud)
        self.lock = threading.Lock()

    def write(self, b):
        with self.lock:
            self.f.write_data(b)

    def read(self):
        with self.lock:
            return self.f.read_data(4096)


class SerialLink:
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial(port, baud, timeout=0)

    def write(self, b):
        self.s.write(b)

    def read(self):
        return self.s.read(4096)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--osc-port", type=int, default=57130)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", default="ftdi://ftdi:0x6015/1", help="ftdi:// URL (default) or serial device")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("-v", "--verbose", action="store_true", help="print every OSC message and event")
    ap.add_argument("--batch-ms", type=float, default=2.0,
                    help="collect events this long into one frame; consecutive values of the same CC/param collapse")
    a = ap.parse_args()

    params = load_params()
    mapper = Mapper(params)
    link = FtdiLink(a.port, a.baud) if a.port.startswith("ftdi://") else SerialLink(a.port, a.baud)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.bind, a.osc_port))
    sock.setblocking(False)
    print(f"[osc_bridge] OSC udp://{a.bind}:{a.osc_port} -> {a.port}  ({len(params)} slot params)", flush=True)

    stats = {"osc": 0, "events": 0, "unknown": 0}
    pending, deadline = [], None

    def key(e):
        """events that may replace the previous one: same CC / bend / bpm / slot param"""
        t, ch, x, y = e[0], e[1], e[2], e[3]
        if t == EV_CC:
            return (t, ch, x)
        if t in (EV_BEND, EV_BPM):
            return (t, ch)
        if t == EV_RAW_PARAM and y not in strobes:
            return (t, x, y)
        return None

    strobes = {params[n][0] for n in ("trig", "gateoff", "kill") if n in params}

    def flush():
        nonlocal pending, deadline
        for k in range(0, len(pending), 16):  # firmware frames hold <= 16 events
            link.write(slip_frame(pending[k:k + 16]))
        stats["events"] += len(pending)
        pending, deadline = [], None

    while True:
        timeout = 0.005 if deadline is None else max(0.0, deadline - time.time())
        r, _, _ = select.select([sock, sys.stdin], [], [], timeout)
        if sock in r:
            try:
                while True:
                    data, src = sock.recvfrom(65536)
                    for addr, args in parse_osc(data):
                        stats["osc"] += 1
                        evs = mapper(addr, args)
                        if evs is None:
                            stats["unknown"] += 1
                            if a.verbose:
                                print(f"[osc_bridge] unmapped {addr} {args}", flush=True)
                            continue
                        if a.verbose:
                            print(f"[osc_bridge] {addr} {args} -> {' '.join(x.hex() for x in evs)}", flush=True)
                        for e in evs:
                            k = key(e)
                            if k is not None and pending and key(pending[-1]) == k:
                                pending[-1] = e      # knob sweep: only the latest value matters
                            else:
                                pending.append(e)
                        if deadline is None:
                            deadline = time.time() + a.batch_ms / 1000.0
            except BlockingIOError:
                pass
        if pending and (time.time() >= deadline or len(pending) >= 16):
            flush()
        if sys.stdin in r:
            line = sys.stdin.readline()
            if not line:
                break
            link.write(line.rstrip("\n").encode() + b"\r")
        d = link.read()
        if d:
            sys.stdout.write(d.decode(errors="replace"))
            sys.stdout.flush()


if __name__ == "__main__":
    main()
