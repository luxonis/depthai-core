#!/bin/bash
# Interactive development shell in a build container. The depthai-core checkout that contains this directory
# (device/rvc4) is mounted at /work; the shell starts in /work/device/rvc4.
#
# Usage: ./start_dev_shell.sh rvc4
#
# Environment: the same as create_deployment_artifacts.sh (DEPTHAI_DEVICE_SDK_IMAGE, DEPTHAI_ACCEPT_QUALCOMM_LICENSE,
# DEPTHAI_DEVICE_QAIRT_ARCHIVE, DEPTHAI_DEVICE_SDK_CACHE, DEPTHAI_DEVICE_RVC4_VENDOR, DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL,
# DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR, HEXAGON_SDK_DIR).

if [ $# -lt 1 ]; then
    echo "Usage: $0 rvc4"
    exit 1
fi
if [ "$1" != "rvc4" ]; then
    echo "Unsupported target '$1', only the rvc4 target is supported"
    exit 1
fi

SOURCE_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
CORE_DIR=$(readlink -f "$SOURCE_DIR/../..")
cd "$SOURCE_DIR"
# The dev image: a development shell builds each target
DEPTHAI_DEVICE_SDK_IMAGE_KIND=dev
source "$SOURCE_DIR/docker/container-env.sh"
mkdir -p "$SOURCE_DIR/build_docker_arm64/home"

docker run \
    -e HOME=/work/device/rvc4/build_docker_arm64/home \
    -e "TERM=xterm-256color" \
    "${CONTAINER_ARGS[@]}" \
    -it --rm "$DEPTHAI_DEVICE_SDK_IMAGE" /bin/bash
