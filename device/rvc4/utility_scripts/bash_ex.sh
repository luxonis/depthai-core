#!/bin/bash
# Opens an interactive shell in the running RVC4 development container (started with ./start_dev_shell.sh rvc4).
# The image is DEPTHAI_DEVICE_SDK_IMAGE, or the default from start_dev_shell.sh.

ROOT_DIR="$(dirname "${BASH_SOURCE[0]}")/.."
DEFAULT_IMAGE=$(grep -oP '^DEPTHAI_DEVICE_SDK_IMAGE_DEFAULT="\K[^"]+' "$ROOT_DIR/start_dev_shell.sh")
IMAGE="${DEPTHAI_DEVICE_SDK_IMAGE:-$DEFAULT_IMAGE}"

CONTAINER=$(docker ps --filter "ancestor=$IMAGE" --format '{{.ID}}' | head -n 1)
if [ -z "$CONTAINER" ]; then
    echo "No running container of image $IMAGE; start one with ./start_dev_shell.sh rvc4" >&2
    exit 1
fi

docker exec -ti -w /work/device/rvc4 "$CONTAINER" /bin/bash
