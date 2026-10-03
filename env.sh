# source this: . ./env.sh
_ULX3S_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$_ULX3S_ROOT/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin:$_ULX3S_ROOT/tools/oss-cad-suite/bin:$PATH"
. "$_ULX3S_ROOT/.venv/bin/activate"
