#!/usr/bin/env bash
# One-time setup for building our MicroPython firmware for the on-board ESP32 (idempotent).
#   ~/esp/esp-idf-v5.2.2    Espressif's ESP-IDF (+ tools in ~/.espressif), the version
#                           MicroPython v1.24.1 builds with
#   ~/esp/micropython       MicroPython v1.24.1 (esp32ecp5 runs on 1.21 .. 1.24)
# ESP-IDF 5.2 wants Python <= 3.12 and the system has 3.14: uv provides a 3.12 for it.
#
#   host/esp32/setup_toolchain.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ESP="$HOME/esp"
IDF="$ESP/esp-idf-v5.2.2"
MPY="$ESP/micropython"
mkdir -p "$ESP"

echo "== Python 3.12 (uv)"
[[ -x "$ROOT/.venv/bin/uv" ]] || "$ROOT/.venv/bin/pip" install -q uv
"$ROOT/.venv/bin/uv" python install 3.12
PY312="$("$ROOT/.venv/bin/uv" python find 3.12)"
PYBIN="$ESP/py312"
mkdir -p "$PYBIN"
ln -sf "$PY312" "$PYBIN/python3"
ln -sf "$PY312" "$PYBIN/python"
export PATH="$PYBIN:$PATH"
python3 --version

echo "== ESP-IDF v5.2.2"
if [[ ! -d "$IDF/.git" ]]; then
    git clone -b v5.2.2 --depth 1 --recursive --shallow-submodules https://github.com/espressif/esp-idf.git "$IDF"
fi
(cd "$IDF" && ./install.sh esp32)

echo "== MicroPython v1.24.1"
if [[ ! -d "$MPY/.git" ]]; then
    git clone -b v1.24.1 --depth 1 https://github.com/micropython/micropython.git "$MPY"
fi
make -C "$MPY/mpy-cross" -j"$(nproc)"
(source "$IDF/export.sh" >/dev/null && make -C "$MPY/ports/esp32" submodules)
echo "== done: source host/esp32/env.sh before building"
