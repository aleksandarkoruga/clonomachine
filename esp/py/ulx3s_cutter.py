# The break cutter app on the ULX3S's ESP32: settings from /cutter.json (knobs, preset,
# conductor, phrase), then the link to the FPGA (the C module "cutter").
import json
import time

import cutter

SETTINGS = "/cutter.json"


def load():
    try:
        with open(SETTINGS) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def apply(s):
    if "preset" in s:
        cutter.preset(s["preset"])
    for k, v in s.get("knobs", {}).items():
        cutter.knob(k, v)
    if "conductor" in s:
        cutter.conductor(s["conductor"])
    if "bars" in s or "sections" in s:
        cutter.phrase(s.get("bars", 0), s.get("sections", 0))


def save(preset=None):
    """The knobs as they are now become the start-up settings."""
    s = {"knobs": {k: cutter.knob(k) for k in cutter.knobs()}, "conductor": cutter.conductor()}
    st = cutter.status()
    s["bars"] = st["bars"]
    with open(SETTINGS, "w") as f:
        json.dump(s, f)


def run():
    time.sleep(1)  # Ctrl-C here keeps the REPL on the serial pins
    apply(load())
    cutter.start()
