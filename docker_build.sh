#!/bin/bash

set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

if [ -d "$BUILD_DIR" ]; then
    echo ">>> Cleaning existing build directory..."
    rm -rf "$BUILD_DIR"
fi

echo ">>> Configuring project..."
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_TOOLCHAIN_FILE=

echo ">>> Building project..."
cmake --build "$BUILD_DIR" -- -j"$(nproc)"

echo ">>> Build complete."
