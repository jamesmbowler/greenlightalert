#!/bin/zsh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" >/dev/null
cmake --build "$BUILD_DIR" --target red_light_box_center_test >/dev/null

"$BUILD_DIR/red_light_box_center_test" "$@"
