#!/usr/bin/env python3
"""Send shell lines to the board's console and print what comes back.

Every line gets a fixed read time, so it also works while the board prints continuously
(unlike ftdi_console.py -s, which can cut the capture short). A line "sleep <s>" only
reads for that long.

  host/console_cmd.py status pool "list 1"
  host/console_cmd.py -w 2.0 "usbwatch 1500"
  host/console_cmd.py "midimon 1" "sleep 5" "midimon 0"

The bridge (osc_bridge.py) must not be running: only one program can hold the board.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ftdi_console import open_console


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lines", nargs="+", help='console lines; "sleep <s>" only reads')
    ap.add_argument("-w", "--wait", type=float, default=1.0, help="read time after each line (s)")
    a = ap.parse_args()
    f = open_console()

    def read(t):
        out, end = b"", time.time() + t
        while time.time() < end:
            d = f.read_data(4096)
            if d:
                out += d
            else:
                time.sleep(0.005)
        return out.decode(errors="replace").replace("\r", "")

    read(0.2)  # drop whatever was pending
    for line in a.lines:
        if line.startswith("sleep "):
            sys.stdout.write(read(float(line.split()[1])))
            continue
        f.write_data(line.encode() + b"\r")
        sys.stdout.write(read(a.wait))
    sys.stdout.flush()
    f.close()


if __name__ == "__main__":
    main()
