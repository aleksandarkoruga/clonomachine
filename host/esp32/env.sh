# source host/esp32/env.sh: ESP-IDF (with its Python 3.12) for building the ESP32 firmware.
# Everything lives in tools/esp/ (see host/esp32/setup_toolchain.sh).
_ESP="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/tools/esp"
export IDF_TOOLS_PATH="$_ESP/espressif"
export UV_CACHE_DIR="$_ESP/cache/uv"
export PIP_CACHE_DIR="$_ESP/cache/pip"
export PATH="$_ESP/bin:$PATH"
source "$_ESP/esp-idf-v5.2.2/export.sh" >/dev/null
export MPY_DIR="$_ESP/micropython"
