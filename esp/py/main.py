# Frozen: starts the break cutter (esp/py/ulx3s_cutter.py). Ctrl-C on the REPL (115200, through
# the pass-through bitstream) within the first second keeps the REPL instead.
import ulx3s_cutter

ulx3s_cutter.run()
