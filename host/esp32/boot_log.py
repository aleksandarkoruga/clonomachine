#!/usr/bin/env python3
"""Reset the on-board ESP32 through the pass-through (host/esp32/esp.sh) and print what it says
at 115200: the ROM's reset / boot-mode line, then whatever its firmware prints.

  host/esp32/boot_log.py           normal boot
  host/esp32/boot_log.py --download   GPIO0 low: ROM download mode
"""
import sys
import time

import pyftdi.serialext

URL = "ftdi://ftdi:0x6015/1"

ser = pyftdi.serialext.serial_for_url(URL, baudrate=115200, timeout=0.1)
download = "--download" in sys.argv
ser.dtr = False
ser.rts = True           # RTS alone: EN low (reset)
time.sleep(0.2)
if download:
    ser.dtr = True       # DTR alone: GPIO0 low
    ser.rts = False
    time.sleep(0.1)
    ser.dtr = False
else:
    ser.rts = False
out = b""
end = time.time() + float(next((a.split("=")[1] for a in sys.argv if a.startswith("--time=")), "2.5"))
while time.time() < end:
    out += ser.read(4096)
print(out.decode(errors="replace"))
ser.close()
