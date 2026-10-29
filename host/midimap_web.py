#!/usr/bin/env python3
"""MIDI map editor in the browser, with the board on USB.

  .venv/bin/python3 host/midimap_web.py        then open http://localhost:8765

Loads the board's active MIDI map (/midimap.json on its card), shows every page as a table of
CC numbers and what they drive, lets you edit it and sends it back: the board checks it, keeps
the old file as /midimap.json.bak, saves and uses the new one at once. With "live MIDI" on, the
rows light up as the Machinedrum's messages arrive.

Talks to the board over the USB console like the other host tools (pyftdi), so nothing else may
hold the console meanwhile (osc_bridge.py, cutter_live.py, console_cmd.py). Standard library only.
"""
import argparse
import json
import os
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ftdi_console import open_console  # noqa: E402

PAGE = os.path.join(HERE, "midimap_web.html")
MIDI_RE = re.compile(r"\[midi ([0-9a-f]{2})\s+(\d+)\s+(\d+)\s+ch (\d+)\]")
CHUNK = 80  # bytes per "mmput" line (160 hex digits; the console takes 191 characters)


class Board:
    """The console: one command at a time; MIDI monitor lines go to the live listeners."""

    def __init__(self):
        self.f = open_console()
        self.f.set_latency_timer(2)
        self.cmd_lock = threading.Lock()
        self.lines = queue.Queue()
        self.listeners = set()
        self.listeners_lock = threading.Lock()
        self.monitor = False
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b""
        while True:
            d = self.f.read_data(4096)
            if not d:
                time.sleep(0.002)
                continue
            buf += d
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").replace("\r", "")
                while line.startswith("smp> "):
                    line = line[5:]
                m = MIDI_RE.search(line)
                if m:
                    ev = {"st": int(m.group(1), 16), "d1": int(m.group(2)), "d2": int(m.group(3)),
                          "ch": int(m.group(4)), "t": time.time()}
                    with self.listeners_lock:
                        for q in list(self.listeners):
                            q.put(ev)
                    continue
                self.lines.put(line)

    def send(self, line):
        self.f.write_data(line.encode() + b"\r")

    def command(self, line, done, timeout=5.0):
        """Sends line, returns the lines read until done(line) is true (that one included)."""
        with self.cmd_lock:
            while not self.lines.empty():
                self.lines.get_nowait()
            self.send(line)
            out, end = [], time.time() + timeout
            while time.time() < end:
                try:
                    ln = self.lines.get(timeout=0.05)
                except queue.Empty:
                    continue
                out.append(ln)
                if done(ln):
                    return out
            raise TimeoutError(f"no answer to '{line.split()[0]}' from the board")

    def block(self, cmd, tag):
        lines = self.command(cmd, lambda ln: ln.strip() == ">>>" + tag)
        start = max(i for i, ln in enumerate(lines) if ln.strip() == "<<<" + tag)
        return "\n".join(lines[start + 1:-1])

    def get_map(self):
        return json.loads(self.block("midimap json", "MIDIMAP"))

    def get_names(self):
        return json.loads(self.block("midimap names", "NAMES"))

    def put_map(self, text):
        data = text.encode()
        if len(data) > 8000:
            return False, "the map is too long for the board (8000 bytes at most)"
        h = 2166136261
        for b in data:
            h = ((h ^ b) * 16777619) & 0xFFFFFFFF
        with_ack = lambda n: (lambda ln: ln.strip() == f"MMPUT {n}" or ln.startswith("MMPUT ERROR"))  # noqa: E731
        self.command("mmput begin", with_ack(0))
        sent = 0
        for i in range(0, len(data), CHUNK):
            part = data[i:i + CHUNK]
            sent += len(part)
            ans = self.command("mmput " + part.hex(), with_ack(sent))
            if ans[-1].startswith("MMPUT ERROR"):
                return False, ans[-1][len("MMPUT ERROR "):]
        ans = self.command(f"mmput end {h:08x}", lambda ln: ln.startswith("MMPUT OK") or ln.startswith("MMPUT ERROR"),
                           timeout=10)
        last = ans[-1]
        return last.startswith("MMPUT OK"), last[len("MMPUT OK "):] if last.startswith("MMPUT OK") else last[12:]

    def defaults(self):
        ans = self.command("midimap defaults", lambda ln: ln.startswith("MIDI: /midimap.json"), timeout=10)
        return ans[-1]

    def set_monitor(self, on):
        if on != self.monitor:
            self.send("midimon %d" % (1 if on else 0))
            self.monitor = on


class MockBoard:
    """--mock: the firmware's defaults (build/host/test_midimap prints them) and the control names
    from fw/midimap.c, random MIDI when live is on. For working on the page without the board."""

    def __init__(self):
        import random
        import subprocess
        root = os.path.dirname(HERE)
        exe = os.path.join(root, "build", "host", "test_midimap")
        subprocess.check_call(["gcc", "-std=c11", "-O2", "-I" + os.path.join(root, "fw"),
                               os.path.join(HERE, "test_midimap.c"), os.path.join(root, "fw", "midimap.c"),
                               os.path.join(root, "fw", "jsonp.c"), "-o", exe])
        out = subprocess.run([exe], capture_output=True, text=True).stdout
        self.map = json.loads(out[:out.index("\n}\n") + 2])
        src = open(os.path.join(root, "fw", "midimap.c")).read()
        grab = lambda name: re.findall(r'"([A-Za-z0-9]+)"', src[src.index(name):src.index("};", src.index(name))])  # noqa: E731
        self.names = {"controls": grab("mm_target_names[MT_NUM]"), "reverb": grab("rk_names[RK_NUM]")}
        self.listeners, self.listeners_lock, self.monitor = set(), threading.Lock(), False
        self.random = random

        def fake():
            while True:
                time.sleep(0.15)
                if not self.monitor:
                    continue
                r = self.random
                ch, st = r.choice([(5, 0xB4), (3, 0xB2), (8, 0xB7), (7, 0xB6), (5, 0x94), (6, 0xE5)])
                ev = {"st": st, "d1": r.randint(16, 30) if st & 0xF0 == 0xB0 else r.randint(0, 12),
                      "d2": r.randint(1, 127), "ch": ch, "t": time.time()}
                with self.listeners_lock:
                    for q in list(self.listeners):
                        q.put(ev)
        threading.Thread(target=fake, daemon=True).start()

    def get_map(self):
        return self.map

    def get_names(self):
        return self.names

    def put_map(self, text):
        self.map = json.loads(text)
        with open(os.path.join(os.path.dirname(HERE), "build", "host", "midimap_mock_put.json"), "w") as f:
            f.write(text)  # exactly what the board would get (check it with the firmware's parser)
        return True, "%d pages (mock)" % len(self.map.get("pages", []))

    def defaults(self):
        return "MIDI: defaults (mock)"

    def set_monitor(self, on):
        self.monitor = on


def make_handler(board):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if self.path in ("/", "/index.html"):
                    with open(PAGE, "rb") as f:
                        return self.reply(200, f.read(), "text/html; charset=utf-8")
                if self.path == "/api/map":
                    return self.reply(200, {"map": board.get_map(), "names": board.get_names()})
                if self.path == "/api/events":
                    return self.events()
                self.reply(404, {"error": "not found"})
            except Exception as e:  # the board didn't answer, the console is busy ...
                self.reply(503, {"error": str(e)})

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(n).decode() if n else ""
                if self.path == "/api/map":
                    json.loads(body)  # at least valid JSON before it goes anywhere
                    ok, msg = board.put_map(body)
                    return self.reply(200 if ok else 400, {"ok": ok, "message": msg})
                if self.path == "/api/defaults":
                    return self.reply(200, {"ok": True, "message": board.defaults()})
                if self.path == "/api/monitor":
                    board.set_monitor(bool(json.loads(body or "{}").get("on")))
                    return self.reply(200, {"ok": True, "monitor": board.monitor})
                self.reply(404, {"error": "not found"})
            except Exception as e:
                self.reply(400, {"ok": False, "message": str(e)})

        def events(self):  # server-sent events: the MIDI the board receives
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = queue.Queue()
            with board.listeners_lock:
                board.listeners.add(q)
            try:
                while True:
                    try:
                        ev = q.get(timeout=10)
                        self.wfile.write(b"data: " + json.dumps(ev).encode() + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with board.listeners_lock:
                    board.listeners.discard(q)

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--mock", action="store_true", help="no board: its defaults, fake MIDI (for the page)")
    a = ap.parse_args()
    board = MockBoard() if a.mock else Board()
    try:
        board.get_names()
    except Exception as e:
        sys.exit(f"the board doesn't answer on the console ({e}); is another tool holding it?")
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(board))
    print(f"MIDI map editor: http://localhost:{a.port}   (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        board.set_monitor(False)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
