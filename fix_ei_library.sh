#!/bin/bash
# Fixes Edge Impulse Arduino library after reinstalling via zip.
# Run this any time you reinstall jbowler-project-1_inferencing.

set -e

LIB="$HOME/Documents/Arduino/libraries/jbowler-project-1_inferencing"

if [ ! -d "$LIB" ]; then
  echo "ERROR: Library not found at $LIB"
  exit 1
fi

# Fix 1: Increase EI_MAX_OVERFLOW_BUFFER_COUNT for ESP32-S3 (default 30 causes
# CMSIS-NN scratch buffer failures, falling back to slow unoptimized paths ~10s inference)
PORTING_H="$LIB/src/edge-impulse-sdk/porting/ei_classifier_porting.h"
sed -i '' 's/^#define EI_MAX_OVERFLOW_BUFFER_COUNT\t30$/#define EI_MAX_OVERFLOW_BUFFER_COUNT\t50/' "$PORTING_H"
echo "✓ EI_MAX_OVERFLOW_BUFFER_COUNT → 50 in ei_classifier_porting.h"

# Fix 2: Increase kTensorArenaSize to match EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE
# (default ~638KB is smaller than required ~765KB, causing heap corruption via
# CMSIS-NN overflow buffers writing past their allocated heap blocks)
COMPILED_CPP="$LIB/src/tflite-model/tflite_learn_790184_19_compiled.cpp"
sed -i '' 's/constexpr int kTensorArenaSize = [^;]*/constexpr int kTensorArenaSize = 3 * 1024 * 1024/g' "$COMPILED_CPP"
echo "✓ kTensorArenaSize → 3MB in tflite_learn_790184_19_compiled.cpp"

# Fix 3: Pad overflow buffer allocations to prevent heap corruption.
# CMSIS-NN underestimates its scratch buffer size, writing past the allocation
# and corrupting the next heap block's header, crashing on free().
sed -i '' 's/ptr = ei_calloc(bytes, 1);/ptr = ei_calloc(bytes + 256, 1);/' "$COMPILED_CPP"
echo "✓ overflow buffer padding +256 bytes in tflite_learn_790184_19_compiled.cpp"

# Clear Arduino build cache so changes take effect on next build
rm -rf "$HOME/Library/Caches/arduino/sketches"
echo "✓ Arduino build cache cleared"

echo ""
echo "Done. Rebuild in Arduino IDE."
