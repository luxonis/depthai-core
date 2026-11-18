#!/bin/bash
# Builds, checks and pushes the public build images of the RVC4 firmware (Dockerfile.sdk in this folder).
# The CI of depthai-device-vendor-fw runs 'build' and 'push' when it changes the public sysroot. The build scripts of
# device/rvc4 and the workflow containers_device_rvc4_sdk.yml use 'pinned' and 'check'.
#
# Usage:
#   sdk-images.sh build <sysroot tarball>  Build the release and the dev image with the public sysroot <tarball>
#                                          (rvc4-public-sysroot-<version>.tar.xz), give them the four tags, check them.
#   sdk-images.sh push <sysroot tarball>   Push the four tags of the images that 'build' made from <tarball>. Stops
#                                          before it pushes anything when the fixed tag is already in the registry.
#   sdk-images.sh tags <sysroot tarball>   Print the four tags (fixed release, fixed dev, OS release, OS dev).
#   sdk-images.sh check [<image>]          Check <image> and <image>-dev. Default: the pinned image (docker pulls it
#                                          when it is not on this host).
#   sdk-images.sh pinned [release|dev]     Print the image that depthai-core pins: DEPTHAI_DEVICE_RVC4_SDK_IMAGE in
#                                          cmake/Depthai/DepthaiDeviceRVC4Config.cmake, with -dev for dev.
#
# Tags. <os> is the Luxonis OS version of the sysroot (the os-release line of its VERSION file), <sha> the first 12
# characters of the sha256 of the tarball:
#   os-<os>-<sha>, os-<os>-<sha>-dev   Fixed: one sysroot. A push never moves them. depthai-core pins these.
#   os-<os>, os-<os>-dev               The newest images for that OS version. Each push moves them.
#
# Environment:
#   DEPTHAI_DEVICE_SDK_REPOSITORY  Image repository for build, push and tags. Default: ghcr.io/luxonis/depthai-device-sdk
#   DEPTHAI_DEVICE_SDK_BUILD_ARGS  More arguments for docker build, split at spaces (for example --no-cache --pull).
set -euo pipefail

DOCKER_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
CORE_DIR=$(readlink -f "$DOCKER_DIR/../../..")
PIN_FILE="$CORE_DIR/cmake/Depthai/DepthaiDeviceRVC4Config.cmake"
REPOSITORY="${DEPTHAI_DEVICE_SDK_REPOSITORY:-ghcr.io/luxonis/depthai-device-sdk}"

usage() {
    sed -n '/^# Usage:/,/^# Tags/p' "${BASH_SOURCE[0]}" | sed '$d; s/^# \{0,1\}//' >&2
    exit 2
}

pinned() {
    local image
    image=$(sed -n 's/^set(DEPTHAI_DEVICE_RVC4_SDK_IMAGE "\([^"]*\)".*/\1/p' "$PIN_FILE")
    if [ -z "$image" ]; then
        echo "error: no DEPTHAI_DEVICE_RVC4_SDK_IMAGE in $PIN_FILE" >&2
        exit 1
    fi
    case "${1:-release}" in
        release) echo "$image" ;;
        dev) echo "$image-dev" ;;
        *) usage ;;
    esac
}

# Sets TARBALL, SYSROOT_SHA256, OS_VERSION and the four tags for the tarball $1
sysroot_info() {
    if [ ! -f "$1" ]; then
        echo "error: no sysroot tarball '$1'" >&2
        exit 1
    fi
    TARBALL=$(readlink -f "$1")
    SYSROOT_SHA256=$(sha256sum "$TARBALL" | cut -d' ' -f1)
    OS_VERSION=$(tar -xJOf "$TARBALL" rvc4-sysroot/VERSION | sed -n 's/^os-release: Luxonis OS RVC4 \([0-9][0-9.]*\) .*/\1/p')
    if [ -z "$OS_VERSION" ]; then
        echo "error: $TARBALL has no rvc4-sysroot/VERSION with an 'os-release: Luxonis OS RVC4 <version>' line" >&2
        exit 1
    fi
    TAG_FIXED="$REPOSITORY:os-$OS_VERSION-${SYSROOT_SHA256:0:12}"
    TAG_FIXED_DEV="$TAG_FIXED-dev"
    TAG_OS="$REPOSITORY:os-$OS_VERSION"
    TAG_OS_DEV="$TAG_OS-dev"
}

# Runs the checks of the images $1 (release) and $2 (dev)
check() {
    local tag
    for tag in "$1" "$2"; do
        echo "Checking $tag"
        docker run --rm "$tag" bash -c '
            set -e
            aarch64-linux-gnu-g++ --version | head -1
            cmake --version | head -1
            python3 -c "import numpy; print(\"numpy\", numpy.__version__)"
            S="$DEPTHAI_DEVICE_RVC4_SYSROOT"
            test -f "$S/usr/include/python3.10/Python.h"
            cat "$S/VERSION"
            test -f "$S/usr/include/fastcv/fastcv.h"
            test -f "$S/usr/share/licenses/fastcv/LICENSE"
            install-hexagon-sdk.sh --help >/dev/null
            # The C headers come from the sysroot, not from a C library of the compiler
            stdio=$(echo "#include <stdio.h>" | aarch64-linux-gnu-gcc --sysroot="$S" -x c -M - | tr " " "\n" | grep "/stdio.h$")
            case "$stdio" in "$S"/*) ;; *) echo "error: stdio.h is not from the sysroot: $stdio" >&2; exit 1 ;; esac
            # A C++ program links against the sysroot with the start files of the OS (as toolchains/rvc4-gnu.cmake)
            # and needs no libstdc++ symbol version newer than the one of the OS (GCC 11: GLIBCXX_3.4.29)
            runtime=$(ls -d "$S"/usr/lib/aarch64-*-linux/* | head -1)
            # (std::condition_variable::wait: GCC 12 gave it the version GLIBCXX_3.4.30)
            printf "#include <condition_variable>\n#include <iostream>\n#include <mutex>\n#include <string>\n#include <thread>\n#include <vector>\nint main() { std::vector<std::string> v{\"a\"}; std::mutex m; std::condition_variable c; bool done = false; std::thread t([&] { std::lock_guard<std::mutex> l(m); done = true; c.notify_one(); }); { std::unique_lock<std::mutex> l(m); c.wait(l, [&] { return done; }); } t.join(); std::cout << v[0] << std::endl; }\n" > /tmp/check.cpp
            aarch64-linux-gnu-g++ --sysroot="$S" -B"$runtime/" -fPIE -pie -O2 -mbranch-protection=standard /tmp/check.cpp -o /tmp/check
            aarch64-linux-gnu-readelf -d /tmp/check | grep -q "NEEDED.*\[libstdc++\.so\.6\]" || { echo "error: the test program does not link the shared libstdc++ of the OS" >&2; exit 1; }
            newest=$(aarch64-linux-gnu-readelf -V /tmp/check | grep -o "GLIBCXX_3\.4\.[0-9]*" | sort -t. -k3 -n | tail -1)
            test "${newest##*.}" -le 29 || { echo "error: the test program needs $newest" >&2; exit 1; }
            echo "compile check: ok (stdio.h from the sysroot, newest libstdc++ symbol version $newest)"'
    done
    # Only the dev image has the sanitizer libraries
    test "$(docker run --rm "$1" aarch64-linux-gnu-g++ -print-file-name=libasan.so)" = libasan.so
    docker run --rm "$2" bash -c 'for l in libasan.so libtsan.so libubsan.so; do test -f "$(aarch64-linux-gnu-g++ -print-file-name=$l)"; done'
    # Both images have the same sysroot
    local release_sha dev_sha
    release_sha=$(docker inspect --format '{{index .Config.Labels "com.luxonis.rvc4.sysroot-sha256"}}' "$1")
    dev_sha=$(docker inspect --format '{{index .Config.Labels "com.luxonis.rvc4.sysroot-sha256"}}' "$2")
    test -n "$release_sha" && test "$release_sha" = "$dev_sha"
    echo "Checked: $1 and $2 (sysroot sha256 $release_sha)"
}

build() {
    sysroot_info "$1"
    local context revision target tag
    context=$(mktemp -d)
    # shellcheck disable=SC2064 # expand now: the variable is local
    trap "rm -rf '$context'" EXIT
    # The build context 'sysroot' has only the tarball, under the name that Dockerfile.sdk expects
    ln "$TARBALL" "$context/rvc4-public-sysroot.tar.xz" 2>/dev/null || cp "$TARBALL" "$context/rvc4-public-sysroot.tar.xz"
    revision=$(git -C "$CORE_DIR" rev-parse HEAD 2>/dev/null || echo unknown)
    echo "Public sysroot: $TARBALL (Luxonis OS $OS_VERSION, sha256 $SYSROOT_SHA256)"
    for target in release dev; do
        if [ "$target" = release ]; then tag="$TAG_FIXED"; else tag="$TAG_FIXED_DEV"; fi
        # shellcheck disable=SC2086 # extra arguments are split at spaces on purpose
        docker build ${DEPTHAI_DEVICE_SDK_BUILD_ARGS:-} -f "$DOCKER_DIR/Dockerfile.sdk" --target "$target" \
            --build-context sysroot="$context" \
            --build-arg RVC4_SYSROOT_SHA256="$SYSROOT_SHA256" \
            --build-arg RVC4_OS_VERSION="$OS_VERSION" \
            --build-arg RVC4_CORE_REVISION="$revision" \
            -t "$tag" "$DOCKER_DIR"
    done
    docker tag "$TAG_FIXED" "$TAG_OS"
    docker tag "$TAG_FIXED_DEV" "$TAG_OS_DEV"
    check "$TAG_FIXED" "$TAG_FIXED_DEV"
    printf 'Built:\n  %s\n  %s\n  %s\n  %s\n' "$TAG_FIXED" "$TAG_FIXED_DEV" "$TAG_OS" "$TAG_OS_DEV"
}

push() {
    sysroot_info "$1"
    local tag sha
    for tag in "$TAG_FIXED" "$TAG_FIXED_DEV"; do
        sha=$(docker inspect --format '{{index .Config.Labels "com.luxonis.rvc4.sysroot-sha256"}}' "$tag" 2>/dev/null || true)
        if [ "$sha" != "$SYSROOT_SHA256" ]; then
            echo "error: no local image $tag of this sysroot; run '$0 build $1' first" >&2
            exit 1
        fi
        if docker manifest inspect "$tag" >/dev/null 2>&1; then
            echo "error: $tag is already in the registry. A fixed tag is never moved: make a new sysroot." >&2
            exit 1
        fi
    done
    for tag in "$TAG_FIXED" "$TAG_FIXED_DEV" "$TAG_OS" "$TAG_OS_DEV"; do
        docker push "$tag"
    done
    echo "Pushed. Pin it in $PIN_FILE:"
    echo "  set(DEPTHAI_DEVICE_RVC4_SDK_IMAGE \"$TAG_FIXED\")"
}

[ $# -ge 1 ] || usage
command=$1
shift
case "$command" in
    build) [ $# -eq 1 ] || usage; build "$1" ;;
    push) [ $# -eq 1 ] || usage; push "$1" ;;
    tags) [ $# -eq 1 ] || usage; sysroot_info "$1"; printf '%s\n' "$TAG_FIXED" "$TAG_FIXED_DEV" "$TAG_OS" "$TAG_OS_DEV" ;;
    check)
        if [ $# -eq 1 ]; then check "$1" "$1-dev"; elif [ $# -eq 0 ]; then check "$(pinned release)" "$(pinned dev)"; else usage; fi ;;
    pinned) [ $# -le 1 ] || usage; pinned "${1:-release}" ;;
    *) usage ;;
esac
