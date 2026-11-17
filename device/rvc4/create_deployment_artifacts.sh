#!/bin/bash
# Non-interactive build of the RVC4 firmware package (fwp) in a build container. The depthai-core checkout that
# contains this directory (device/rvc4) is mounted at /work.
#
# Usage:  ./create_deployment_artifacts.sh <rvc4|rvc4-asan-ubsan|rvc4-tsan>
# Output: device/rvc4/build_docker_arm64_<target>/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz
#         (plus depthai-device-rvc4-fwp-LICENSE and the SPDX / CycloneDX SBOMs next to it)
#
# Environment:
#   DEPTHAI_DEVICE_SDK_IMAGE  Build image. Default: the public image ghcr.io/luxonis/depthai-device-sdk:<sysroot version
#                             in docker/Dockerfile.sdk> for rvc4, and the same tag with -dev (it adds the sanitizer
#                             libraries) for rvc4-asan-ubsan and rvc4-tsan.
#   DEPTHAI_ACCEPT_QUALCOMM_LICENSE
#                             yes: the first build with the public image downloads the parts of QAIRT (Qualcomm AI
#                             Runtime SDK) that the build uses from Qualcomm, about 10 MB, and you accept the license
#                             terms of that SDK (README.md, Public build container).
#   DEPTHAI_DEVICE_QAIRT_ARCHIVE
#                             The QAIRT archive (v<version>.zip) that you downloaded from Qualcomm, in place of the download.
#   DEPTHAI_DEVICE_SDK_CACHE  Folder on the host for the caches that all checkouts share: the compiler cache (ccache), the
#                             binary cache of the vcpkg dependencies and the QAIRT parts in use.
#                             Default: ~/.cache/depthai-device-sdk
#   CCACHE_MAXSIZE            Size limit of the compiler cache. Default: 10G
#   DEPTHAI_DEVICE_RVC4_VENDOR
#                             A local prebuilt vendor archive (depthai-device-rvc4-vendor-<version>[-asan-ubsan|-tsan].tar.xz,
#                             the licensee-only code of the firmware), in place of the download. Default: the build
#                             downloads the archive of DEPTHAI_DEVICE_RVC4_VENDOR_VERSION
#                             (../../cmake/Depthai/DepthaiDeviceRVC4Config.cmake).
#   DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL
#                             Base URL of that download. Default: https://artifacts.luxonis.com/artifactory
#   DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR
#                             Sources of the vendor module (repository depthai-device-vendor-fw; its CI and Luxonis licensees),
#                             mounted at /depthai-device-rvc4-vendor, in place of the prebuilt archive. Not set:
#                             the build uses the prebuilt archive.
#   HEXAGON_SDK_DIR           Qualcomm Hexagon SDK on the host, mounted at /opt/hexagon-sdk: the build then builds the DSP
#                             offload from source. Without it, the DSP offload comes from the prebuilt archive (the
#                             Luxonis OS SDK image has the SDK).
#   DEPTHAI_DEVICE_DOCKER_RUN_ARGS
#                             More arguments for docker run, split at spaces (for example --network host).
#   DEPTHAI_BUILD_JOBS        Number of parallel compile jobs. Default: half of the CPU cores.
#   DEPTHAI_DEVICE_DEBUG_INFO 1: compile with debug data, for a debugger and for the file and line of an address in a
#                             crash report. 0: compile without it. The firmware package is the same, the build is
#                             faster and its build folder is 10 times smaller. Default: 0 for the target rvc4 with the
#                             prebuilt private module, 1 for the sanitizer targets and for a build with the sources
#                             of the private module.
#   DEPTHAI_DEVICE_BUILD_CACHE
#                             0: do not use the published build cache. Default: a build of the target rvc4 without
#                             debug data downloads the object files and the vcpkg dependencies that a CI job
#                             published for the commit of the checkout (or for the nearest of its 10 parent commits
#                             that has them), and compiles only the rest (README.md, Build time and caches).
#   DEPTHAI_DEVICE_BUILD_CACHE_BASE_URL
#                             Base URL of the published build caches. Default: the Luxonis artifact server.
#   SOURCE_DATE_EPOCH         The build time that the firmware reports, in seconds since 1970. Default: the time of the
#                             commit of the checkout, so that two builds of one commit are equal.
#   VCPKG_MAX_CONCURRENCY     Number of parallel jobs for the vcpkg dependency builds. Default: all CPU cores.

if [ $# -lt 1 ]; then
    echo "Usage: $0 <rvc4|rvc4-asan-ubsan|rvc4-tsan>"
    exit 1
fi
if [[ "$1" != *"rvc4"* ]]; then
    echo "Unsupported target '$1', only RVC4 targets (rvc4, rvc4-asan-ubsan, rvc4-tsan) are supported"
    exit 1
fi

SOURCE_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
CORE_DIR=$(readlink -f "$SOURCE_DIR/../..")
cd "$SOURCE_DIR"
echo "Building target '$1' from $SOURCE_DIR (depthai-core: $CORE_DIR)"
if [ "$1" = rvc4 ]; then DEPTHAI_DEVICE_SDK_IMAGE_KIND=release; else DEPTHAI_DEVICE_SDK_IMAGE_KIND=dev; fi
source "$SOURCE_DIR/docker/container-env.sh"

# HOME for the non-interactive build: the tools write their caches there, and the home directory of the image is
# not writable for the host user that runs the build.
mkdir -p "$SOURCE_DIR/build_docker_arm64/home"

docker run \
    -e CI \
    -e DEPTHAI_BUILD_JOBS \
    -e VCPKG_MAX_CONCURRENCY \
    -e HOME=/work/device/rvc4/build_docker_arm64/home \
    "${CONTAINER_ARGS[@]}" \
    --rm "$DEPTHAI_DEVICE_SDK_IMAGE" /work/device/rvc4/docker/build.sh "$1"
