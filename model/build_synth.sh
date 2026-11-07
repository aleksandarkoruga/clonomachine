#!/usr/bin/env bash
# Builds the synth model's command-line tool into build/synth/synth_render.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/build/synth"
g++ -O3 -march=native -std=c++20 -Wall -Wextra -Wno-unused-parameter \
    "$ROOT/model/synth_render.cpp" "$ROOT/model/synth_engine.cpp" "$ROOT/model/synth_patch.cpp" \
    "$ROOT/model/wt_tables.cpp" -o "$ROOT/build/synth/synth_render"
g++ -O3 -march=native -std=c++20 -Wall -Wextra "$ROOT/model/test_synth_osc.cpp" "$ROOT/model/wt_tables.cpp" \
    -o "$ROOT/build/synth/test_synth_osc"
