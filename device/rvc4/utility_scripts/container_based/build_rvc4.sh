#!/bin/bash

cd /work/device/rvc4

source /prepare-env.sh

# this should be run inside container (./attach.sh)
cmake --build /work/device/rvc4/build_docker_arm64_rvc4 --parallel $((($(nproc) + 1) / 2)) --target depthai-device-rvc4 --config RelWithDebInfo
