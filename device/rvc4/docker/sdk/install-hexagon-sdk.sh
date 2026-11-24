#!/bin/bash
# Install the Qualcomm Hexagon SDK into the build container at /opt/Hexagon_SDK, to build the DSP offload of
# ImageAlign and StereoDepth from source. Without it the firmware takes the DSP offload from the prebuilt archive
# of the private module (the dsp/ folder).
# The SDK is not redistributable by Luxonis: install it with Qualcomm Package Manager (qpm-cli), accept
# Qualcomm's license terms, and run this script with the installed SDK directory (e.g. ~/Qualcomm/Hexagon_SDK/5.x.y.z).
set -euo pipefail

usage() {
    cat <<USAGE
usage: $(basename "$0") <hexagon-sdk-dir>
  hexagon-sdk-dir  the installed Hexagon SDK directory (contains build/cmake/hexagon_fun.cmake and tools/HEXAGON_Tools)
Destination: \${HEXAGON_SDK_INSTALL_DIR:-/opt/Hexagon_SDK}
USAGE
}

if [ $# -ne 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
    [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] && exit 0
    exit 2
fi
sdk_dir=$1
dest=${HEXAGON_SDK_INSTALL_DIR:-/opt/Hexagon_SDK}

[ -d "$sdk_dir" ] || { echo "error: $sdk_dir is not a directory" >&2; exit 1; }
[ -f "$sdk_dir/build/cmake/hexagon_fun.cmake" ] || { echo "error: $sdk_dir/build/cmake/hexagon_fun.cmake not found: not a Hexagon SDK root" >&2; exit 1; }
if [ ! -d "$sdk_dir/tools/HEXAGON_Tools" ]; then
    echo "warning: $sdk_dir/tools/HEXAGON_Tools not found; the Hexagon compiler tools are part of the SDK installation" >&2
fi

if [ "$(cd "$sdk_dir" && pwd -P)" = "$(mkdir -p "$dest" && cd "$dest" && pwd -P)" ]; then
    echo "$sdk_dir is already $dest"
else
    echo "Copying the Hexagon SDK from $sdk_dir to $dest (several GB) ..."
    mkdir -p "$dest"
    cp -a "$sdk_dir"/. "$dest"/
fi
du -sh "$dest" 2>/dev/null || true
if [ "${HEXAGON_SDK_ROOT:-}" = "$dest" ]; then
    echo "Done. HEXAGON_SDK_ROOT already points at $dest (set by the build image)."
else
    echo "Done. Before configuring the firmware:"
    echo "  export HEXAGON_SDK_ROOT=$dest"
fi
echo "The firmware build then builds the DSP offload from source."
echo "Reminder: the Hexagon SDK is Qualcomm software; by installing it you accepted Qualcomm's license terms, which also govern the resulting firmware image."
