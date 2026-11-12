#!/bin/bash
# Attaches gdb (from the SDK) to the firmware that runs under gdbserver on the device. Run it in the development
# container.
#
# Usage: DEVICE_IP=<device-ip> ./debug.sh
# Beforehand run on the host:
#   ./utility_scripts/build.sh && DEVICE_IP=<device-ip> ./utility_scripts/deploy.sh
# and then on the device:
#   /data/device_based/debug.sh
# and then launch a depthai-core example on the host, with the watchdog disabled:
#   export DEPTHAI_WATCHDOG=0

if [ -z "$DEVICE_IP" ]; then
    echo "Usage: DEVICE_IP=<device-ip> $0" >&2
    exit 1
fi

source /usr/local/oe-sdk-hardcoded-buildpath/environment-setup-aarch64-oe-linux
$GDB \
  -ex "set sysroot /usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux" \
  -ex "add-auto-load-safe-path /usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux/usr/lib/libstdc++.so.6.0.29-gdb.py" \
  -ex "target remote $DEVICE_IP:2000" \
  /work/device/rvc4/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4
