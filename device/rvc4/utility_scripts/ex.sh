#!/bin/bash
# Runs a command in the running RVC4 development container (started with ./start_dev_shell.sh rvc4).
# Usage: ./utility_scripts/ex.sh "<command>"
# The image is DEPTHAI_DEVICE_SDK_IMAGE, or the default of start_dev_shell.sh (docker/container-env.sh).

ROOT_DIR="$(dirname "${BASH_SOURCE[0]}")/.."
DEFAULT_IMAGE="ghcr.io/luxonis/depthai-device-sdk:$(sed -n 's/^ARG RVC4_SYSROOT_VERSION=//p' "$ROOT_DIR/docker/Dockerfile.sdk")-dev"
IMAGE="${DEPTHAI_DEVICE_SDK_IMAGE:-$DEFAULT_IMAGE}"

CONTAINER=$(docker ps --filter "ancestor=$IMAGE" --format '{{.ID}}' | head -n 1)
if [ -z "$CONTAINER" ]; then
    echo "No running container of image $IMAGE; start one with ./start_dev_shell.sh rvc4" >&2
    exit 1
fi

docker exec -w /work/device/rvc4 -t "$CONTAINER" /bin/bash -c "$@"
