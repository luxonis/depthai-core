#!/bin/bash
if [ $# -lt 1 ]
then
echo "target argument does not exist"
exit 1
else
echo "######################At least one argument exists"
fi

# Build tools: the public SDK image ships them; the legacy private image needs an install.
missing=""
for t in ninja ccache curl zip unzip tar; do command -v "$t" >/dev/null 2>&1 || missing="$missing $t"; done
if [ -n "$missing" ]; then
    echo "Installing missing build tools:$missing"
    sudo apt-get update && sudo apt-get install -y ninja-build ccache curl zip unzip tar
fi
SOURCE_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
ROOT_DIR="$SOURCE_DIR/../"
cd "$ROOT_DIR"
BUILD_DIR="$PWD/build_docker_arm64"

BUILD_TYPE="RelWithDebInfo"

valid_build_types=("Release" "RelWithDebInfo" "Debug" "MinSizeRel")

for item in "${valid_build_types[@]}"
do
    if [ "$1" == "$item" ]; then
        BUILD_TYPE=$1
        break
    fi
done


export VCPKG_DEFAULT_BINARY_CACHE=$BUILD_DIR/vcpkg_cache
export X_VCPKG_REGISTRIES_CACHE="${BUILD_DIR}/vcpkg_cache/registries"
mkdir -p $X_VCPKG_REGISTRIES_CACHE
export CCACHE_DIR=$BUILD_DIR/ccache

if [[ "$1" == *"rvc4"* ]]; then
    BUILD_DIR="$PWD/build_docker_arm64_$1"
    echo "RVC4"
    if [[ "$1" == *"asan-ubsan"* ]]; then
        echo "ASAN/UBSAN"
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D DEPTHAI_ASAN_UBSAN=ON
    elif [[ "$1" == *"tsan"* ]]; then
        echo "TSAN"
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D DEPTHAI_TSAN=ON
    else
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE
    fi
    cmake --build "$BUILD_DIR" --parallel "${DEPTHAI_BUILD_JOBS:-$((($(nproc) + 1) / 2))}" --target depthai-device-rvc4 --config $BUILD_TYPE
else
    echo "Unsupported target '$1', only RVC4 targets (rvc4, rvc4-asan-ubsan, rvc4-tsan) are supported"
    exit 1
fi
