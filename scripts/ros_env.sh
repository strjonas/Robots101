#!/bin/bash
set -eo pipefail

ROOT_DIR="${ROBOTS101_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROOT_DIR="$(cd "$ROOT_DIR" && pwd)"
: "${CONDA_PREFIX:=$ROOT_DIR/.pixi/envs/default}"

link_short_plugin_dir() {
  local source_dir="$1"
  local short_dir="$2"
  local target_path

  mkdir -p "$short_dir"
  for plugin_path in "$source_dir"/*.dylib; do
    [ -e "$plugin_path" ] || continue
    target_path="$short_dir/$(basename "$plugin_path")"
    if [ -e "$target_path" ] || [ -L "$target_path" ]; then
      continue
    fi
    ln -s "$plugin_path" "$target_path" 2>/dev/null || true
  done
}

export ROBOTS101_ROOT="$ROOT_DIR"
export CONDA_PREFIX
export ROS_HOME="${ROS_HOME:-$ROOT_DIR/.ros}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"

mkdir -p "$ROS_HOME"

source "$CONDA_PREFIX/setup.sh"

if [ -f "$ROOT_DIR/install/setup.bash" ]; then
  source "$ROOT_DIR/install/setup.bash"
fi

if [ "$(uname)" = "Darwin" ] && [ -d "$CONDA_PREFIX/lib/OGRE-Next" ]; then
  link_short_plugin_dir "$CONDA_PREFIX/lib/OGRE" /tmp/robots101-ogre
  link_short_plugin_dir "$CONDA_PREFIX/lib/OGRE-Next" /tmp/robots101-ogre2
  export OGRE_RESOURCE_PATH=/tmp/robots101-ogre
  export OGRE2_RESOURCE_PATH=/tmp/robots101-ogre2
fi
