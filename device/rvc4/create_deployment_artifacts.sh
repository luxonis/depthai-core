#!/bin/bash
# Non-interactive build of the RVC4 firmware package (fwp) inside the Luxonis OS SDK container.
# The depthai-core checkout that contains this directory (device/rvc4) is mounted at /work.
#
# Usage:  ./create_deployment_artifacts.sh <rvc4|rvc4-asan-ubsan|rvc4-tsan>
# Output: device/rvc4/build_docker_arm64_<target>/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
#         (plus depthai-device-rvc4-fwp-LICENSE and the SPDX / CycloneDX SBOMs next to it)
#
# Environment:
#   DEPTHAI_DEVICE_SDK_IMAGE  Docker image with the Luxonis OS RVC4 SDK. Required.
#   DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR
#                             Checkout of the private depthai-device-rvc4-vendor module (licensee-only Qualcomm/ST code). It is
#                             mounted at /depthai-device-rvc4-vendor, where CMake finds it (DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR).
#                             Default: a checkout of the private repository depthai-device-vendor-fw
#                             next to the depthai-core checkout.
#   DEPTHAI_BUILD_JOBS        Number of parallel compile jobs. Default: half of the CPU cores.
#   VCPKG_MAX_CONCURRENCY     Number of parallel jobs for the vcpkg dependency builds. Default: all CPU cores.
#   HEXAGON_SDK_DIR           Qualcomm Hexagon SDK on the host, for an image without one (the Luxonis OS SDK image has it).
#                             Mounted at /opt/hexagon-sdk and passed to the build as HEXAGON_SDK_ROOT.

if [ $# -lt 1 ]; then
    echo "Usage: $0 <rvc4|rvc4-asan-ubsan|rvc4-tsan>"
    exit 1
fi

SOURCE_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
CORE_DIR=$(readlink -f "$SOURCE_DIR/../..")

if [ -z "${DEPTHAI_DEVICE_SDK_IMAGE:-}" ]; then
    echo "Set DEPTHAI_DEVICE_SDK_IMAGE to the Luxonis OS RVC4 SDK image that you have access to" >&2
    exit 1
fi
VENDOR_SOURCE_DIR="${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR:-$CORE_DIR/../depthai-device-vendor-fw}"

# Move to source directory
cd "$SOURCE_DIR"

if [[ "$1" != *"rvc4"* ]]; then
    echo "Unsupported target '$1', only RVC4 targets (rvc4, rvc4-asan-ubsan, rvc4-tsan) are supported"
    exit 1
fi

echo "Building target '$1' from $SOURCE_DIR (depthai-core: $CORE_DIR) with image $DEPTHAI_DEVICE_SDK_IMAGE"

VENDOR_MOUNT=()
if [ -d "$VENDOR_SOURCE_DIR" ]; then
    VENDOR_SOURCE_DIR=$(readlink -f "$VENDOR_SOURCE_DIR")
    echo "Private module: $VENDOR_SOURCE_DIR (mounted at /depthai-device-rvc4-vendor)"
    VENDOR_MOUNT=(-v "$VENDOR_SOURCE_DIR":/depthai-device-rvc4-vendor -e DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR=/depthai-device-rvc4-vendor)
else
    echo "WARNING: private module depthai-device-rvc4-vendor not found at '$VENDOR_SOURCE_DIR' (set DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR); the firmware build needs it" >&2
fi

HEXAGON_MOUNT=()
if [ -n "${HEXAGON_SDK_DIR:-}" ]; then
    HEXAGON_SDK_DIR=$(readlink -f "$HEXAGON_SDK_DIR")
    echo "Hexagon SDK: $HEXAGON_SDK_DIR (mounted at /opt/hexagon-sdk)"
    HEXAGON_MOUNT=(-v "$HEXAGON_SDK_DIR":/opt/hexagon-sdk -e HEXAGON_SDK_ROOT=/opt/hexagon-sdk)
fi

# git inside the container: a git worktree keeps its metadata in the main checkout, outside the mounted tree.
# Mount that directory at the same path, read-only, so that the build can read the commit and the submodules.
GIT_MOUNT=()
GIT_COMMON_DIR=$(cd "$CORE_DIR" && readlink -f "$(git rev-parse --git-common-dir 2>/dev/null)" 2>/dev/null)
case "$GIT_COMMON_DIR" in
    ""|"$CORE_DIR"/*) ;;
    *) GIT_MOUNT=(-v "$GIT_COMMON_DIR":"$GIT_COMMON_DIR":ro) ;;
esac

# HOME for the non-interactive build: the tools write their caches there, and the home directory of the image is
# not writable for the host user that runs the build.
mkdir -p "$SOURCE_DIR/build_docker_arm64/home"

# Mounts passwd&shadow and adds group 'sudo' (27) to the user, so that sudo in docker works as the current user
# (sudoers is overwritten in the image already).
docker run \
    -e CI \
    -e DEPTHAI_BUILD_JOBS \
    -e VCPKG_MAX_CONCURRENCY \
    -e HOME=/work/device/rvc4/build_docker_arm64/home \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/shadow:/etc/shadow:ro \
    -v "$CORE_DIR":/work \
    "${GIT_MOUNT[@]}" \
    "${VENDOR_MOUNT[@]}" \
    "${HEXAGON_MOUNT[@]}" \
    -w /work/device/rvc4 \
    --user="$(id -u):$(id -g)" \
    --group-add 27 \
    --rm "$DEPTHAI_DEVICE_SDK_IMAGE" /work/device/rvc4/docker/build.sh "$1"
