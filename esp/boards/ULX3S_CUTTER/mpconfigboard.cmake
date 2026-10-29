# The ULX3S's on-board ESP32 (ESP32-D0WD-V3, PSRAM, 16 MB flash) running the break cutter.
set(IDF_TARGET esp32)
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    ${SDKCONFIG_IDF_VERSION_SPECIFIC}
    boards/sdkconfig.spiram
    boards/sdkconfig.240mhz
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)
