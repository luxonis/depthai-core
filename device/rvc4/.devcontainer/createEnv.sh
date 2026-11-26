#!/bin/bash
# Writes .devcontainer/.env (gitignored) for docker-compose.yml: the container user, the SDK image and the vendor
# module settings. VS Code runs it as the initializeCommand of the dev container.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR=$( cd -- "$SCRIPT_DIR/.." &> /dev/null && pwd )
CORE_DIR=$( cd -- "$REPO_DIR/../.." &> /dev/null && pwd )

DOCKER_USER=$(id -u):$(id -g)

# Build image: the Luxonis OS SDK image. The dev container uses its environment script, sudo and debugger, which the
# public build image (README.md, Public build container) does not have. Same default as in Dockerfile.rvc4.
DEPTHAI_DEVICE_SDK_IMAGE="${DEPTHAI_DEVICE_SDK_IMAGE:?Set DEPTHAI_DEVICE_SDK_IMAGE to the Luxonis OS RVC4 SDK image you have access to (README.md, Public build container)}"

# Vendor module (licensee-only Qualcomm/ST code). Default: CMake downloads its prebuilt archive. DEPTHAI_DEVICE_RVC4_VENDOR:
# a local archive below $HOME (the container mounts $HOME at the same path). DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR:
# its sources (Luxonis licensees), mounted at /depthai-device-rvc4-vendor.
VENDOR_SOURCE_DIR="${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR:-}"
VENDOR_SOURCE_DIR_IN_CONTAINER=""
if [ -n "$VENDOR_SOURCE_DIR" ] && [ -d "$VENDOR_SOURCE_DIR" ]; then
    VENDOR_SOURCE_DIR=$( cd -- "$VENDOR_SOURCE_DIR" &> /dev/null && pwd )
    VENDOR_SOURCE_DIR_IN_CONTAINER=/depthai-device-rvc4-vendor
else
    if [ -n "$VENDOR_SOURCE_DIR" ]; then
        echo "WARNING: vendor module sources not found at '$VENDOR_SOURCE_DIR' (DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR); the build uses the prebuilt archive" >&2
    fi
    # docker-compose cannot skip a volume, so an empty directory is mounted instead and the container still starts
    VENDOR_SOURCE_DIR="$REPO_DIR/build_docker_arm64/depthai-device-rvc4-vendor-missing"
    mkdir -p "$VENDOR_SOURCE_DIR"
fi
VENDOR_ARCHIVE=""
if [ -n "${DEPTHAI_DEVICE_RVC4_VENDOR:-}" ]; then
    VENDOR_ARCHIVE=$(readlink -f "$DEPTHAI_DEVICE_RVC4_VENDOR")
fi

{
    echo "DOCKER_USER=$DOCKER_USER"
    echo "DEPTHAI_DEVICE_SDK_IMAGE=$DEPTHAI_DEVICE_SDK_IMAGE"
    echo "VENDOR_SOURCE_DIR=$VENDOR_SOURCE_DIR"
    echo "DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR=$VENDOR_SOURCE_DIR_IN_CONTAINER"
    echo "DEPTHAI_DEVICE_RVC4_VENDOR=$VENDOR_ARCHIVE"
} > "$SCRIPT_DIR/.env"
