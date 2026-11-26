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


# Build caches. The caller can set them: docker/container-env.sh sets the caches that all checkouts share.
# Default (the caller sets none): in the build folder.
export VCPKG_DEFAULT_BINARY_CACHE=${VCPKG_DEFAULT_BINARY_CACHE:-$BUILD_DIR/vcpkg_cache}
export X_VCPKG_REGISTRIES_CACHE="$VCPKG_DEFAULT_BINARY_CACHE/registries"
mkdir -p "$VCPKG_DEFAULT_BINARY_CACHE" "$X_VCPKG_REGISTRIES_CACHE"
export CCACHE_DIR=${CCACHE_DIR:-$BUILD_DIR/ccache}

if [[ "$1" == *"rvc4"* ]]; then
    BUILD_DIR="$PWD/build_docker_arm64_$1"
    echo "RVC4"
    if [[ "$1" == *"asan-ubsan"* ]]; then
        echo "ASAN/UBSAN"
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D DEPTHAI_DEVICE_DEBUG_INFO="${DEPTHAI_DEVICE_DEBUG_INFO:-ON}" -D DEPTHAI_ASAN_UBSAN=ON
    elif [[ "$1" == *"tsan"* ]]; then
        echo "TSAN"
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D DEPTHAI_DEVICE_DEBUG_INFO="${DEPTHAI_DEVICE_DEBUG_INFO:-ON}" -D DEPTHAI_TSAN=ON
    else
        cmake -G "Ninja Multi-Config" -S . -B "$BUILD_DIR" -D DEPTHAI_ENABLE_CURL=0 -D HUNTER_ROOT="$BUILD_DIR/.hunter" -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D DEPTHAI_DEVICE_DEBUG_INFO="${DEPTHAI_DEVICE_DEBUG_INFO:-ON}"
    fi || exit 1 # no build after a failed configure step: it would build an older configuration or hide the reason
    if [ "${DEPTHAI_DEVICE_CONFIGURE_ONLY:-0}" = 1 ]; then
        echo "Configure step only (DEPTHAI_DEVICE_CONFIGURE_ONLY=1): $BUILD_DIR"
        exit 0
    fi
    JOBS="${DEPTHAI_BUILD_JOBS:-$((($(nproc) + 1) / 2))}"
    # A CI job publishes object files as a compiler cache for the builds of users (ci/device-rvc4-build-cache.sh).
    # These object files are compiled first, with DEPTHAI_DEVICE_BUILD_CACHE_OUT as a second storage of ccache:
    # depthai-core and its Python bindings, then each firmware source that includes no header of a Qualcomm SDK
    # (published-cache-objects.py has the reason and the rule). The check stops the job when a published object file
    # used such a header. The other firmware sources are compiled after that, without the second storage.
    if [ -n "${DEPTHAI_DEVICE_BUILD_CACHE_OUT:-}" ]; then
        export CCACHE_SECONDARY_STORAGE="file:$DEPTHAI_DEVICE_BUILD_CACHE_OUT"
        cmake --build "$BUILD_DIR" --parallel "$JOBS" --target depthai-core depthai --config $BUILD_TYPE || exit 1
        "$SOURCE_DIR/published-cache-objects.py" list "$BUILD_DIR" "$BUILD_TYPE" "$JOBS" > "$BUILD_DIR/published-cache-objects.txt" || exit 1
        if [ -s "$BUILD_DIR/published-cache-objects.txt" ]; then
            xargs -a "$BUILD_DIR/published-cache-objects.txt" ninja -C "$BUILD_DIR" -f "build-$BUILD_TYPE.ninja" -j "$JOBS" || exit 1
        fi
        unset CCACHE_SECONDARY_STORAGE
        "$SOURCE_DIR/published-cache-objects.py" check "$BUILD_DIR" "$BUILD_TYPE" || exit 1
    fi
    cmake --build "$BUILD_DIR" --parallel "$JOBS" --target depthai-device-rvc4 --config $BUILD_TYPE
else
    echo "Unsupported target '$1', only RVC4 targets (rvc4, rvc4-asan-ubsan, rvc4-tsan) are supported"
    exit 1
fi
