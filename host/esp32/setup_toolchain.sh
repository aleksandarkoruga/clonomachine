#!/usr/bin/env bash
# One-time setup for building our MicroPython firmware for the on-board ESP32 (idempotent).
# Everything stays inside tools/esp/ in this project (delete that folder and it's all gone):
#   tools/esp/esp-idf-v5.2.2   Espressif's ESP-IDF, the version MicroPython v1.24.1 builds with
#   tools/esp/espressif        its compilers and its own Python venv (IDF_TOOLS_PATH)
#   tools/esp/micropython      MicroPython v1.24.1 (esp32ecp5 runs on 1.21 .. 1.24)
#   tools/esp/python           Python 3.12 from uv (ESP-IDF 5.2 wants <= 3.12, the system has 3.14)
#   tools/esp/cache            uv / pip download caches
# uv itself is in the project's .venv.
#
#   host/esp32/setup_toolchain.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ESP="$ROOT/tools/esp"
IDF="$ESP/esp-idf-v5.2.2"
MPY="$ESP/micropython"
export IDF_TOOLS_PATH="$ESP/espressif"
export UV_PYTHON_INSTALL_DIR="$ESP/python"
export UV_CACHE_DIR="$ESP/cache/uv"
export PIP_CACHE_DIR="$ESP/cache/pip"
mkdir -p "$ESP"

echo "== Python 3.12 (uv, inside tools/esp)"
[[ -x "$ROOT/.venv/bin/uv" ]] || "$ROOT/.venv/bin/pip" install -q uv
"$ROOT/.venv/bin/uv" python install 3.12
PY312="$("$ROOT/.venv/bin/uv" python find 3.12)"
mkdir -p "$ESP/bin"
ln -sf "$PY312" "$ESP/bin/python3"
ln -sf "$PY312" "$ESP/bin/python"
export PATH="$ESP/bin:$PATH"
python3 --version

echo "== ESP-IDF v5.2.2"
if [[ ! -d "$IDF/.git" ]]; then
    git clone -b v5.2.2 --depth 1 --recursive --shallow-submodules https://github.com/espressif/esp-idf.git "$IDF"
else # an interrupted clone: finish the submodules
    git -C "$IDF" submodule update --init --recursive --depth 1
fi
(cd "$IDF" && ./install.sh esp32)

echo "== MicroPython v1.24.1"
if [[ ! -d "$MPY/.git" ]]; then
    git clone -b v1.24.1 --depth 1 https://github.com/micropython/micropython.git "$MPY"
fi
# gcc 15 errors on a char-array initialiser MicroPython 1.24 uses: allow it
make -C "$MPY/mpy-cross" -j"$(nproc)" CFLAGS_EXTRA=-Wno-unterminated-string-initialization
(source "$IDF/export.sh" >/dev/null && make -C "$MPY/ports/esp32" submodules)
echo "== done: source host/esp32/env.sh before building"
