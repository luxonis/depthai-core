#!/bin/bash
# Builds the firmware in the development container, copies it to the device and starts it under gdbserver.
# Used by the VS Code task "var-deploy-gdb-rvc4" (.vscode/tasks.json) before a remote debugging session.
#
# Usage: ./remote_build_and_run_rvc4.sh [<env-file>] [--config <CMake config>]
#   <env-file>  File that sets BOARD_IP=<device-ip>; the VS Code task "Get IP" writes it to docker/boardIP.
#               Optional when DEVICE_IP is set in the environment.
#   DEVICE_IP   IP address of the device; takes precedence over BOARD_IP from the env file.

echo "Deploying to target"

SOURCE_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
BUILD_DIR="$SOURCE_DIR/build_docker_arm64_rvc4"

# Optional first argument: the env file with BOARD_IP
if [ -n "$1" ] && [ "${1:0:2}" != "--" ]; then
    ENV_SCRIPT=$1
    if [ -f "$ENV_SCRIPT" ]; then
        source "$ENV_SCRIPT"
    else
        echo "WARNING: env file '$ENV_SCRIPT' does not exist" >&2
    fi
    shift  # Remove the env file from the argument list
fi

# Default values
CONFIG="RelWithDebInfo"

# Parse options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --config) CONFIG="$2"; shift ;;
        *) break ;;
    esac
    shift
done

DEVICE_IP="${DEVICE_IP:-$BOARD_IP}"
if [ -z "$DEVICE_IP" ]; then
    echo "ERROR: no device IP. Set DEVICE_IP=<device-ip> or pass an env file that sets BOARD_IP=<device-ip>" >&2
    exit 1
fi

echo "Using configuration $CONFIG"

SSH_OPTIONS="-o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"

echo "Running with device IP $DEVICE_IP"
GDBSERVER_PORT="1234"

set -e
# Build
cmake --build "$BUILD_DIR" --parallel 10 --target depthai-device-rvc4 --config $CONFIG


set +e
# Check if gdbserver is available on device
echo "Checking if gdbserver is available on device"
ssh $SSH_OPTIONS root@$DEVICE_IP "gdbserver --version"

# If not, download it and copy it over to the local bin folder
if [ $? -ne 0 ]; then
    echo "gdbserver not found on device, downloading it"
    wget https://github.com/hugsy/gdb-static/raw/master/gdbserver-8.1.1-aarch64-le -O $BUILD_DIR/gdbserver
    ssh $SSH_OPTIONS root@$DEVICE_IP mount -o rw,remount /
    scp $SSH_OPTIONS $BUILD_DIR/gdbserver root@$DEVICE_IP:/usr/bin/gdbserver
    ssh $SSH_OPTIONS root@$DEVICE_IP chmod +x /usr/bin/gdbserver
    ssh $SSH_OPTIONS root@$DEVICE_IP mount -o ro,remount /
fi

set -e
# Stop gate if it is running
echo "Stopping depthai_gate. Remember to start it again (systemctl start depthai_gate) when you are done."
ssh $SSH_OPTIONS root@$DEVICE_IP "systemctl stop depthai_gate"

ssh $SSH_OPTIONS root@$DEVICE_IP "rm -rf /home/root/debug_fw && mkdir /home/root/debug_fw && killall -q gdbserver || true"

scp -r $SSH_OPTIONS $BUILD_DIR/$CONFIG/depthai-device-rvc4 root@$DEVICE_IP:/home/root/debug_fw/depthai-device

echo "Starting GDB Server on Target"
# Run
ssh -t $SSH_OPTIONS root@$DEVICE_IP "cd debug_fw && export ADSP_LIBRARY_PATH='/usr/lib;/dsp;/opt/qcom/aistack/qairt/2.23.0.24.06.24/lib/hexagon-v73/unsigned/' && gdbserver --once :$GDBSERVER_PORT ./depthai-device"
