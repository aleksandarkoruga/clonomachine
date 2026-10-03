#!/usr/bin/env python3
"""ULX3S console over libusb (pyftdi): works right after openFPGALoader, which leaves the
FT231X in bit-bang mode and confuses the kernel tty.

  ftdi_console.py [-t secs] [-s "line"]...     capture / send lines
  ftdi_console.py -i                           interactive shell
"""
import argparse, select, sys, time
from pyftdi.ftdi import Ftdi

URL = "ftdi://ftdi:0x6015/1"

def open_console(baud=1000000):
    f = Ftdi()
    f.open_from_url(URL)
    f.set_bitmode(0, Ftdi.BitMode.RESET)
    f.set_baudrate(baud)
    f.set_line_property(8, 1, "N")
    f.set_flowctrl("")
    f.set_latency_timer(2)
    return f

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=5.0)
    ap.add_argument("-b", type=int, default=1000000)
    ap.add_argument("-s", action="append", default=[], help="line to send (after --delay)")
    ap.add_argument("--delay", type=float, default=0.5)
    ap.add_argument("-i", action="store_true", help="interactive: type shell lines, Ctrl-D quits")
    a = ap.parse_args()
    f = open_console(a.b)
    t0 = time.time(); sent = False
    while a.i or time.time() - t0 < a.t:
        if a.i and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            f.write_data(line.rstrip("\n").encode() + b"\r")
        if not sent and time.time() - t0 > a.delay:
            for l in a.s:
                f.write_data(l.encode() + b"\r"); time.sleep(0.25)
            sent = True
        d = f.read_data(4096)
        if d:
            sys.stdout.write(d.decode(errors="replace")); sys.stdout.flush()
        else:
            time.sleep(0.01)
    f.close()

if __name__ == "__main__":
    main()
