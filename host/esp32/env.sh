# source host/esp32/env.sh: ESP-IDF (with its Python 3.12) for building the ESP32 firmware
export PATH="$HOME/esp/py312:$PATH"
source "$HOME/esp/esp-idf-v5.2.2/export.sh" >/dev/null
export MPY_DIR="$HOME/esp/micropython"
