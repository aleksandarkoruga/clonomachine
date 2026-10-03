#!/usr/bin/env python3
"""Program the ULX3S (optional), serial-boot the firmware and act as console.

Uses pyftdi (libusb), because openFPGALoader leaves the FT231X in bit-bang mode and the
kernel tty sees nothing afterwards. Speaks the LiteX BIOS SFL serial-boot protocol.

  host/load_fw.py --bit build/ulx3s_sampler/gateware/radiona_ulx3s.bit \
                  --fw fw/sampler.bin  [-t 15] [-s "status"]... [-i]
"""
import argparse
import os
import select
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "deps", "litex"))
from litex.tools.litex_term import SFLFrame, sfl_magic_req, sfl_magic_ack, sfl_cmd_load, sfl_cmd_jump

sys.path.insert(0, os.path.dirname(__file__))
from ftdi_console import open_console

MAIN_RAM = 0x40000000
CHUNK = 96  # frame < 128 bytes: fits the BIOS UART ring even when the CPU lags at 1 Mbaud


class Link:
    def __init__(self, f):
        self.f = f
        self.rx = b""

    def pump(self, echo=True):
        d = self.f.read_data(4096)
        if d:
            self.rx += d
            if echo:
                sys.stdout.write(d.decode(errors="replace"))
                sys.stdout.flush()
        return d

    def wait_for(self, token, timeout, echo=True):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if token in self.rx:
                self.rx = self.rx[self.rx.index(token) + len(token):]
                return True
            if not self.pump(echo):
                time.sleep(0.002)
        return False

    def frame(self, cmd, payload, retries=8):
        fr = SFLFrame()
        fr.cmd = cmd
        fr.payload = payload
        enc = fr.encode()
        for _ in range(retries):
            self.rx = b""
            self.f.write_data(enc)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                self.pump(echo=False)
                if self.rx:
                    break
            if self.rx[:1] == b"K":
                self.rx = self.rx[1:]
                return
            if self.rx[:1] != b"C":  # C = CRC error: retry
                raise RuntimeError("SFL reply %r" % self.rx[:8])
        raise RuntimeError("too many SFL retries")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bit", help="bitstream to load into SRAM first")
    ap.add_argument("--fw", default=os.path.join(os.path.dirname(__file__), "..", "fw", "sampler.bin"))
    ap.add_argument("-t", type=float, default=10.0, help="console capture time after boot")
    ap.add_argument("-s", action="append", default=[], help="console line to send after boot")
    ap.add_argument("-i", action="store_true", help="interactive console (stdin) after boot")
    ap.add_argument("--skip-sd", action="store_true", help="answer the firmware's skip-SD prompt")
    ap.add_argument("--late", action="append", default=[], help="console line sent 2 s before the end")
    a = ap.parse_args()

    if a.bit:
        r = subprocess.run(["openFPGALoader", "-b", "ulx3s", a.bit], capture_output=True, text=True)
        if r.returncode:
            sys.exit(r.stdout + r.stderr)
    link = Link(open_console())
    print("[load_fw] waiting for BIOS serial boot request ...")
    if not link.wait_for(sfl_magic_req, 15):
        sys.exit("\n[load_fw] no serial boot request (reset the board / reload the bitstream)")
    link.f.write_data(sfl_magic_ack)
    data = open(a.fw, "rb").read()
    t0 = time.time()
    for off in range(0, len(data), CHUNK):
        link.frame(sfl_cmd_load, (MAIN_RAM + off).to_bytes(4, "big") + data[off:off + CHUNK])
    link.frame(sfl_cmd_jump, MAIN_RAM.to_bytes(4, "big"))
    print("[load_fw] %d bytes in %.1f s, booting" % (len(data), time.time() - t0))
    if a.skip_sd and link.wait_for(b"skip the SD card", 5):
        link.f.write_data(b"s")

    t0 = time.time()
    sent = False
    late_sent = False
    while a.i or time.time() - t0 < a.t:
        if not late_sent and time.time() - t0 > a.t - 2.0:
            for l in a.late:
                link.f.write_data(l.encode() + b"\r")
            late_sent = True
        if not sent and time.time() - t0 > 3.0:
            for l in a.s:
                link.f.write_data(l.encode() + b"\r")
                t1 = time.time()
                while time.time() - t1 < 0.4:
                    link.pump()
            sent = True
        if not link.pump():
            time.sleep(0.005)
        if a.i and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            link.f.write_data(line.rstrip("\n").encode() + b"\r")


if __name__ == "__main__":
    main()
