#!/bin/bash
set -eo pipefail

ROOT_DIR="${ROBOTS101_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROOT_DIR="$(cd "$ROOT_DIR" && pwd)"
: "${CONDA_PREFIX:=$ROOT_DIR/.pixi/envs/default}"

export ROBOTS101_ROOT="$ROOT_DIR"
export CONDA_PREFIX
export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"

source "$CONDA_PREFIX/setup.sh"

if [ -f "$CONDA_PREFIX/etc/conda/activate.d/libgz-rendering8_activate.sh" ]; then
  source "$CONDA_PREFIX/etc/conda/activate.d/libgz-rendering8_activate.sh"
fi

cd "$ROOT_DIR"

colcon build \
  --packages-up-to brain_interfaces brain_nodes brain_bringup \
  --cmake-force-configure \
  --cmake-args \
  "-DCMAKE_SHARED_LINKER_FLAGS=-L$CONDA_PREFIX/lib" \
  "-DCMAKE_MODULE_LINKER_FLAGS=-L$CONDA_PREFIX/lib" \
  "-DCMAKE_EXE_LINKER_FLAGS=-L$CONDA_PREFIX/lib" \
  --event-handlers console_direct+
