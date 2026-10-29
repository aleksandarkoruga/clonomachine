# Frozen into the firmware: the port's usual modules + our app (main.py runs it).
include("$(PORT_DIR)/boards/manifest.py")
module("main.py", base_path="$(BOARD_DIR)/../../py")
module("ulx3s_cutter.py", base_path="$(BOARD_DIR)/../../py")
