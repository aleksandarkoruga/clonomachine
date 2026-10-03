#!/usr/bin/env python3
"""Program the board, abort the BIOS boot sequence and run BIOS commands.
   bios_cmd.py [--bit file] cmd1 cmd2 ...   (prints the console)"""
import argparse, os, subprocess, sys, time
sys.path.insert(0, os.path.dirname(__file__))
from ftdi_console import open_console
ap = argparse.ArgumentParser(); ap.add_argument("--bit"); ap.add_argument("--wait", type=float, default=3.0)
ap.add_argument("cmds", nargs="*"); a = ap.parse_args()
if a.bit:
    subprocess.run(["openFPGALoader", "-b", "ulx3s", a.bit], capture_output=True, check=True)
f = open_console(); rx = b""; t0 = time.time()
while b"sL5DdSMmkekro" not in rx and time.time() - t0 < 15:
    rx += f.read_data(4096)
f.write_data(b"Q")
def run(cmd, wait):
    out = b""; f.write_data(cmd.encode() + b"\r"); t = time.time()
    while time.time() - t < wait:
        d = f.read_data(4096)
        if d: out += d
        else: time.sleep(0.01)
    sys.stdout.write(out.decode(errors="replace").replace("\r", "")); sys.stdout.flush()
run("", 1.0)
for c in a.cmds: run(c, a.wait)
