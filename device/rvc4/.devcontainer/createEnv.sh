#!/bin/bash
# Writes .devcontainer/.env (gitignored) for docker-compose.yml: the container user, the SDK image and the path of
# the private depthai-device-rvc4-vendor module. VS Code runs it as the initializeCommand of the dev container.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR=$( cd -- "$SCRIPT_DIR/.." &> /dev/null && pwd )
CORE_DIR=$( cd -- "$REPO_DIR/../.." &> /dev/null && pwd )

DOCKER_USER=$(id -u):$(id -g)

# Luxonis OS RVC4 SDK image. Today the private Luxonis OS SDK image; the public image
# ghcr.io/luxonis/depthai-device-sdk will replace it once it is published. Same default as in Dockerfile.rvc4.
DEPTHAI_DEVICE_SDK_IMAGE="${DEPTHAI_DEVICE_SDK_IMAGE:?Set DEPTHAI_DEVICE_SDK_IMAGE to the Luxonis OS RVC4 SDK image you have access to (Luxonis staff: the private Luxonis OS SDK image; the public image ghcr.io/luxonis/depthai-device-sdk is planned)}"

# Private depthai-device-rvc4-vendor module (licensee-only Qualcomm/ST code), mounted at /depthai-device-rvc4-vendor.
# Default: a checkout of the private repository depthai-device-vendor-fw next to the depthai-core checkout.
VENDOR_SOURCE_DIR="${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR:-$CORE_DIR/../depthai-device-vendor-fw}"
if [ -d "$VENDOR_SOURCE_DIR" ]; then
    VENDOR_SOURCE_DIR=$( cd -- "$VENDOR_SOURCE_DIR" &> /dev/null && pwd )
else
    # docker-compose cannot skip a volume, so an empty directory is mounted instead and the container still starts
    echo "WARNING: private module depthai-device-rvc4-vendor not found at '$VENDOR_SOURCE_DIR' (set DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR); mounting an empty directory" >&2
    VENDOR_SOURCE_DIR="$REPO_DIR/build_docker_arm64/depthai-device-rvc4-vendor-missing"
    mkdir -p "$VENDOR_SOURCE_DIR"
fi

{
    echo "DOCKER_USER=$DOCKER_USER"
    echo "DEPTHAI_DEVICE_SDK_IMAGE=$DEPTHAI_DEVICE_SDK_IMAGE"
    echo "VENDOR_SOURCE_DIR=$VENDOR_SOURCE_DIR"
} > "$SCRIPT_DIR/.env"
