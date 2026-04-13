#!/bin/zsh
set -euo pipefail

ROOT="/Users/jamesbowler/esp32"
INPUT_DIR="${1:-$ROOT/tests/end_to_end/input}"
OUTPUT_DIR="${2:-$ROOT/tests/end_to_end/out}"
BUILD_DIR="$ROOT/build"

cmake -S "$ROOT" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" --target end_to_end
"$BUILD_DIR/end_to_end" "$INPUT_DIR" "$OUTPUT_DIR"

