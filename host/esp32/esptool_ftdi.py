#!/usr/bin/env python3
"""esptool through the ULX3S FT231X (pyftdi, since openFPGALoader leaves the chip in bit-bang
mode and the kernel tty doesn't see it). Needs the pass-through bitstream (host/esp32/esp.sh).

  host/esp32/esptool_ftdi.py chip_id
  host/esp32/esptool_ftdi.py --baud 460800 write_flash 0x0 app.bin
"""
import sys

import pyftdi.serialext  # noqa: F401  registers ftdi:// URLs with pyserial
import esptool
import esptool.loader

# the "tight" reset drives DTR/RTS with ioctls on a tty file descriptor, which a pyftdi port
# doesn't have: the classic DTR/RTS sequence does the same through pyserial
esptool.loader.UnixTightReset = esptool.loader.ClassicReset

URL = "ftdi://ftdi:0x6015/1"

if __name__ == "__main__":
    args = sys.argv[1:]
    if "--port" not in args and "-p" not in args:
        args = ["--port", URL] + args
    sys.exit(esptool.main(args))
