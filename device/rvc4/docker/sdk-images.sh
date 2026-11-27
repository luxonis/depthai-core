#!/bin/bash
# Builds, checks and pushes the public build images of the RVC4 firmware (Dockerfile.sdk in this folder).
#
# The inputs of the images of one tag are the public sysroot of Luxonis OS and the build files (this folder, the
# LICENSE and the commit of depthai-core) of one commit of the vendor repository. The private CI of the vendor module
# makes the sysroot, builds and checks the images, and uploads the inputs to the artifact server (job public-sysroot):
# <inputs URL>/<tag>/rvc4-public-sysroot.tar.xz and <inputs URL>/<tag>/sdk-image-context.tar.xz. From these inputs,
# 'local' builds the images on each host, and the workflow containers_device_rvc4_sdk.yml pushes them ('publish',
# 'promote'). The build scripts of device/rvc4 use 'ensure': a pinned image that the registry does not have yet is
# built on the host.
#
# Usage:
#   sdk-images.sh build <sysroot tarball>  Build the release and the dev image with the public sysroot <tarball>
#                                          (rvc4-public-sysroot-<version>.tar.xz) and the build files of this
#                                          checkout, give them the fixed tags, check them.
#   sdk-images.sh tags <sysroot tarball>   Print the four tags (fixed release, fixed dev, OS release, OS dev).
#   sdk-images.sh fetch [<tag>]            Download the inputs of <tag> from the artifact server into the cache, check
#                                          their sha256 and print the folder. Default: the pinned tag.
#   sdk-images.sh local [<tag>]            Build the release and the dev image of <tag> on this host from its inputs
#                                          (as 'fetch'), check them. Default: the pinned tag.
#   sdk-images.sh ensure [release|dev]     Print the pinned image (as 'pinned'). When it is not on this host, docker
#                                          pulls it. When the registry does not have it, 'local' builds it.
#   sdk-images.sh publish [<tag>]          When the registry has neither image of <tag>: build them as 'local' and push
#                                          the two fixed tags. Default: the pinned tag.
#   sdk-images.sh promote [<tag>]          Move os-<os> and os-<os>-dev in the registry to the images of <tag>, when
#                                          they point to other images. Default: the pinned tag.
#   sdk-images.sh check [<image>]          Check <image> and <image>-dev. Default: the pinned image (docker pulls it
#                                          when it is not on this host).
#   sdk-images.sh pinned [release|dev]     Print the image that depthai-core pins: DEPTHAI_DEVICE_RVC4_SDK_IMAGE in
#                                          cmake/Depthai/DepthaiDeviceRVC4Config.cmake, with -dev for dev.
#
# Tags. <os> is the Luxonis OS version of the sysroot (the os-release line of its VERSION file), <vendor> the commit of
# the vendor repository that made the sysroot (12 characters, the "tool: make-public-sysroot.py, revision" line of its
# VERSION file). The sha256 of the tarball is the label com.luxonis.rvc4.sysroot-sha256 of the images. <tag> above is
# os-<os>-<vendor>, with or without the repository.
#   os-<os>-<vendor>, os-<os>-<vendor>-dev   Fixed: the inputs of one vendor commit. A push never moves them.
#                                            depthai-core pins these.
#   os-<os>, os-<os>-dev                     The images that the branch develop of depthai-core pins ('promote').
#
# Environment:
#   DEPTHAI_DEVICE_SDK_REPOSITORY  Image repository. Default: ghcr.io/luxonis/depthai-device-sdk
#   DEPTHAI_DEVICE_SDK_INPUTS_URL  Folder of the inputs on the artifact server. Default:
#                                  https://artifacts.luxonis.com/artifactory/luxonis-rvc4-snapshot-local/depthai-device-rvc4/sdk-image
#   DEPTHAI_DEVICE_SDK_CACHE       Cache folder of the build scripts: 'fetch' keeps the inputs in <cache>/sdk-image/<tag>.
#                                  Default: ~/.cache/depthai-device-sdk
#   DEPTHAI_DEVICE_SDK_BUILD_ARGS  More arguments for docker build, split at spaces (for example --no-cache --pull).
set -euo pipefail

DOCKER_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
CORE_DIR=$(readlink -f "$DOCKER_DIR/../../..")
PIN_FILE="$CORE_DIR/cmake/Depthai/DepthaiDeviceRVC4Config.cmake"
REPOSITORY="${DEPTHAI_DEVICE_SDK_REPOSITORY:-ghcr.io/luxonis/depthai-device-sdk}"
INPUTS_URL="${DEPTHAI_DEVICE_SDK_INPUTS_URL:-https://artifacts.luxonis.com/artifactory/luxonis-rvc4-snapshot-local/depthai-device-rvc4/sdk-image}"
CACHE_DIR="${DEPTHAI_DEVICE_SDK_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/depthai-device-sdk}"

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

# sha256 of the file $1 (macOS has no sha256sum)
sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

# Prints the tag os-<os>-<vendor> of $1, a fixed tag with or without the repository. Default: the pinned tag.
tag_name() {
    local tag
    tag="${1:-$(pinned release)}"
    tag="${tag##*:}"
    if ! [[ "$tag" =~ ^os-[0-9][0-9.]*-[0-9a-f]{12}$ ]]; then
        echo "error: '$tag' is not a fixed tag os-<OS version>-<vendor commit, 12 characters>" >&2
        exit 1
    fi
    echo "$tag"
}

# Sets TARBALL, SYSROOT_SHA256, OS_VERSION, VENDOR_REVISION and the four tags for the tarball $1. A tarball of a vendor
# checkout with changes (revision <commit>-dirty) gives tags for a local test; 'publish' never pushes one.
sysroot_info() {
    if [ ! -f "$1" ]; then
        echo "error: no sysroot tarball '$1'" >&2
        exit 1
    fi
    TARBALL=$(readlink -f "$1")
    SYSROOT_SHA256=$(sha256 "$TARBALL")
    local version
    version=$(tar -xJOf "$TARBALL" rvc4-sysroot/VERSION)
    OS_VERSION=$(sed -n 's/^os-release: Luxonis OS RVC4 \([0-9][0-9.]*\) .*/\1/p' <<< "$version")
    if [ -z "$OS_VERSION" ]; then
        echo "error: $TARBALL has no rvc4-sysroot/VERSION with an 'os-release: Luxonis OS RVC4 <version>' line" >&2
        exit 1
    fi
    VENDOR_REVISION=$(sed -n 's/^tool: make-public-sysroot\.py, revision \([^ ]*\)$/\1/p' <<< "$version")
    if ! [[ "$VENDOR_REVISION" =~ ^[0-9a-f]{12}(-dirty)?$ ]]; then
        echo "error: rvc4-sysroot/VERSION of $TARBALL has no vendor commit (line 'tool: make-public-sysroot.py, revision <12 characters>', found '${VENDOR_REVISION:-nothing}')" >&2
        exit 1
    fi
    TAG_FIXED="$REPOSITORY:os-$OS_VERSION-$VENDOR_REVISION"
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
            echo "compile check: ok (stdio.h from the sysroot, newest libstdc++ symbol version $newest)"
            # The notices of the image (sdk/write-notices.sh) list each installed Ubuntu package
            N=/usr/share/doc/depthai-device-sdk
            for f in NOTICE.md ubuntu-packages.tsv LICENSE-depthai-core; do test -s "$N/$f"; done
            diff <(dpkg-query -W -f="\${Package}\n" | sort) <(tail -n +2 "$N/ubuntu-packages.tsv" | cut -f1 | sort)
            echo "notices: ok ($(($(wc -l < "$N/ubuntu-packages.tsv") - 1)) Ubuntu packages in $N/ubuntu-packages.tsv)"'
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

# Builds the release and the dev image of the sysroot tarball $1 with the build files in the folder $2 (Dockerfile.sdk
# and sdk/), the depthai-core license file $3 and the depthai-core commit $4, gives them the fixed tags, checks them
build_images() {
    sysroot_info "$1"
    local docker_dir=$2 license=$3 revision=$4 context target tag
    context=$(mktemp -d)
    # shellcheck disable=SC2064 # expand now: the variable is local
    trap "rm -rf '$context'" EXIT
    # The build context 'sysroot' has only the tarball, under the name that Dockerfile.sdk expects. The build context
    # 'legal' has only the LICENSE file of depthai-core, for the notices of the images.
    mkdir "$context/sysroot" "$context/legal"
    ln "$TARBALL" "$context/sysroot/rvc4-public-sysroot.tar.xz" 2>/dev/null \
        || cp "$TARBALL" "$context/sysroot/rvc4-public-sysroot.tar.xz"
    cp "$license" "$context/legal/LICENSE"
    echo "Public sysroot: $TARBALL (Luxonis OS $OS_VERSION, vendor commit $VENDOR_REVISION, sha256 $SYSROOT_SHA256)"
    echo "Build files: $docker_dir (depthai-core commit $revision)"
    for target in release dev; do
        if [ "$target" = release ]; then tag="$TAG_FIXED"; else tag="$TAG_FIXED_DEV"; fi
        # shellcheck disable=SC2086 # extra arguments are split at spaces on purpose
        docker build ${DEPTHAI_DEVICE_SDK_BUILD_ARGS:-} -f "$docker_dir/Dockerfile.sdk" --target "$target" \
            --build-context sysroot="$context/sysroot" \
            --build-context legal="$context/legal" \
            --build-arg RVC4_SYSROOT_SHA256="$SYSROOT_SHA256" \
            --build-arg RVC4_OS_VERSION="$OS_VERSION" \
            --build-arg RVC4_CORE_REVISION="$revision" \
            --build-arg RVC4_VENDOR_REVISION="$VENDOR_REVISION" \
            -t "$tag" "$docker_dir"
    done
    check "$TAG_FIXED" "$TAG_FIXED_DEV"
    printf 'Built:\n  %s\n  %s\n' "$TAG_FIXED" "$TAG_FIXED_DEV"
}

# Downloads the inputs of the tag $1 into the cache and prints the folder: rvc4-public-sysroot.tar.xz and
# sdk-image-context/ (docker/, LICENSE, REVISION), and 'complete'. The inputs are downloaded, checked and unpacked in a
# staging folder, and one rename makes it the cache folder: a cache folder never changes after that, also when two
# fetches of one tag run at the same time, and a failed fetch leaves no folder. The callers run fetch in a command
# substitution, where bash turns off set -e: each step checks its result.
fetch() {
    local tag dir url file want have stage
    tag=$(tag_name "${1:-}") || exit 1
    dir="$CACHE_DIR/sdk-image/$tag"
    if [ -f "$dir/complete" ]; then
        echo "$dir"
        return
    fi
    mkdir -p "$CACHE_DIR/sdk-image" || exit 1
    stage=$(mktemp -d "$CACHE_DIR/sdk-image/.$tag.XXXXXX") || exit 1
    # shellcheck disable=SC2064 # expand now: the variable is local
    trap "rm -rf '$stage'" EXIT
    url="$INPUTS_URL/$tag"
    echo "Inputs of $tag: $url" >&2
    for file in rvc4-public-sysroot.tar.xz sdk-image-context.tar.xz; do
        if ! curl -fsSL --retry 3 -o "$stage/$file.sha256" "$url/$file.sha256"; then
            echo "error: no $url/$file on the artifact server. The private CI of the vendor module uploads the inputs of a tag (job public-sysroot)." >&2
            exit 1
        fi
        if ! curl -fsSL --retry 3 -o "$stage/$file" "$url/$file"; then
            echo "error: cannot download $url/$file" >&2
            exit 1
        fi
        want=$(cut -d' ' -f1 < "$stage/$file.sha256" | tr -d '[:space:]')
        have=$(sha256 "$stage/$file")
        if [ -z "$want" ] || [ "$want" != "$have" ]; then
            echo "error: the sha256 of $url/$file is $have, the artifact server gives ${want:-nothing}" >&2
            exit 1
        fi
    done
    if ! tar -xJf "$stage/sdk-image-context.tar.xz" -C "$stage"; then
        echo "error: cannot unpack $url/sdk-image-context.tar.xz" >&2
        exit 1
    fi
    for file in docker/Dockerfile.sdk LICENSE REVISION; do
        if [ ! -f "$stage/sdk-image-context/$file" ]; then
            echo "error: $url/sdk-image-context.tar.xz has no sdk-image-context/$file" >&2
            exit 1
        fi
    done
    touch "$stage/complete" || exit 1
    # A cache folder without 'complete' is a leftover (no fetch makes one): replace it. When another fetch makes the
    # folder first, its folder stays and this staging folder goes (also when it was moved into that folder).
    if [ -e "$dir" ] && [ ! -f "$dir/complete" ]; then
        rm -rf "$dir"
    fi
    [ -e "$dir" ] || mv "$stage" "$dir" 2>/dev/null || true
    rm -rf "${dir:?}/$(basename "$stage")"
    if [ ! -f "$dir/complete" ]; then
        echo "error: cannot make the cache folder $dir" >&2
        exit 1
    fi
    echo "$dir"
}

# Builds the images of the tag $1 on this host from its inputs
local_build() {
    local tag dir
    tag=$(tag_name "${1:-}")
    dir=$(fetch "$tag")
    sysroot_info "$dir/rvc4-public-sysroot.tar.xz"
    if [ "$TAG_FIXED" != "$REPOSITORY:$tag" ]; then
        echo "error: the inputs of $tag on the artifact server have the sysroot of ${TAG_FIXED##*:}" >&2
        exit 1
    fi
    build_images "$dir/rvc4-public-sysroot.tar.xz" "$dir/sdk-image-context/docker" "$dir/sdk-image-context/LICENSE" \
        "$(cat "$dir/sdk-image-context/REVISION")"
}

# Prints the digest of the image $1 in the registry, or nothing when the registry does not have it. Stops on other
# errors: 'publish' must not take an error of the registry for a missing tag and push a fixed tag again.
registry_digest() {
    local out
    if out=$(docker buildx imagetools inspect --format '{{.Manifest.Digest}}' "$1" 2>&1); then
        echo "$out"
    elif [[ "$out" == *"not found"* ]]; then
        return 0
    else
        echo "error: cannot read $1 from the registry: $out" >&2
        exit 1
    fi
}

# Prints the pinned image $1 (release or dev) and makes sure that it is on this host: docker pulls it, or, when the
# registry does not have it, it is built here from its inputs. All other output goes to stderr.
ensure() {
    local image log
    image=$(pinned "$1")
    if docker image inspect "$image" >/dev/null 2>&1; then
        echo "$image"
        return
    fi
    log=$(mktemp)
    if docker pull "$image" 2>&1 | tee "$log" >&2; then
        rm -f "$log"
        echo "$image"
        return
    fi
    if ! grep -q "manifest unknown" "$log"; then
        rm -f "$log"
        echo "error: cannot pull $image" >&2
        exit 1
    fi
    rm -f "$log"
    echo "The registry does not have $image yet: building it on this host from its inputs on the artifact server" >&2
    # Under the name of the pin, also when DEPTHAI_DEVICE_SDK_REPOSITORY names another repository
    REPOSITORY="${image%:*}"
    local_build "$(pinned release)" >&2
    echo "$image"
}

publish() {
    local tag ref release_digest dev_digest
    tag=$(tag_name "${1:-}")
    ref="$REPOSITORY:$tag"
    release_digest=$(registry_digest "$ref")
    dev_digest=$(registry_digest "$ref-dev")
    if [ -n "$release_digest" ] && [ -n "$dev_digest" ]; then
        echo "The registry has $ref and $ref-dev: nothing to push"
        return
    fi
    if [ -n "$release_digest" ] || [ -n "$dev_digest" ]; then
        echo "error: the registry has only one of $ref and $ref-dev. A fixed tag is never moved: check the registry." >&2
        exit 1
    fi
    local_build "$tag"
    docker push "$ref"
    docker push "$ref-dev"
    echo "Pushed $ref and $ref-dev"
}

promote() {
    local tag suffix fixed moving want have
    tag=$(tag_name "${1:-}")
    for suffix in "" -dev; do
        fixed="$REPOSITORY:$tag$suffix"
        moving="$REPOSITORY:${tag%-*}$suffix"
        want=$(registry_digest "$fixed")
        if [ -z "$want" ]; then
            echo "error: the registry does not have $fixed; run '$0 publish $tag' first" >&2
            exit 1
        fi
        have=$(registry_digest "$moving")
        if [ "$have" = "$want" ]; then
            echo "$moving is already $fixed ($want)"
            continue
        fi
        # --prefer-index=false: a copy of the manifest, with the same digest, not a new index around it
        docker buildx imagetools create --prefer-index=false --tag "$moving" "$fixed"
        echo "Moved $moving to $fixed ($want)"
    done
}

[ $# -ge 1 ] || usage
command=$1
shift
case "$command" in
    build)
        [ $# -eq 1 ] || usage
        build_images "$1" "$DOCKER_DIR" "$CORE_DIR/LICENSE" "$(git -C "$CORE_DIR" rev-parse HEAD 2>/dev/null || echo unknown)" ;;
    tags) [ $# -eq 1 ] || usage; sysroot_info "$1"; printf '%s\n' "$TAG_FIXED" "$TAG_FIXED_DEV" "$TAG_OS" "$TAG_OS_DEV" ;;
    fetch) [ $# -le 1 ] || usage; fetch "${1:-}" ;;
    local) [ $# -le 1 ] || usage; local_build "${1:-}" ;;
    ensure) [ $# -le 1 ] || usage; ensure "${1:-release}" ;;
    publish) [ $# -le 1 ] || usage; publish "${1:-}" ;;
    promote) [ $# -le 1 ] || usage; promote "${1:-}" ;;
    check)
        if [ $# -eq 1 ]; then check "$1" "$1-dev"; elif [ $# -eq 0 ]; then check "$(pinned release)" "$(pinned dev)"; else usage; fi ;;
    pinned) [ $# -le 1 ] || usage; pinned "${1:-release}" ;;
    *) usage ;;
esac
