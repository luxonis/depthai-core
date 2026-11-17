#!/bin/bash
# Copies the built firmware, its environment script, the Hexagon skeleton libraries and the device-side helper
# scripts to an RVC4 device (into /data).
#
# Usage: DEVICE_IP=<device-ip> ./utility_scripts/deploy.sh
#    or: ./utility_scripts/deploy.sh <device-ip>
# Run ./utility_scripts/build.sh beforehand.

DEVICE_IP="${1:-$DEVICE_IP}"
if [ -z "$DEVICE_IP" ]; then
    echo "Usage: DEVICE_IP=<device-ip> $0   or   $0 <device-ip>" >&2
    exit 1
fi

BUILD_FOLDER="$( dirname "${BASH_SOURCE[0]}" )/../build_docker_arm64_rvc4"

scp -r \
  "${BUILD_FOLDER}/RelWithDebInfo/depthai-device-rvc4" \
  "${BUILD_FOLDER}/scripts/env_set.sh" \
  "${BUILD_FOLDER}/libImageAlignDsp_skel.so" \
  "${BUILD_FOLDER}/libStereoDepthDsp_skel.so" \
  "$( dirname "${BASH_SOURCE[0]}" )/device_based" \
  root@"$DEVICE_IP":/data/
