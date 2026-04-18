#!/bin/bash
source venv/bin/activate

export DEPTHAI_DEVICE_RVC4_FWP=~/src/depthai-device-kb/build_docker_arm64_rvc4/Release/depthai-device-rvc4-fwp.tar.xz
export PYTHONPATH=~/src/depthai-device-kb/external/depthai-core/build/bindings/python/

#python GPUStereo/gpu_stereo.py "$@"
python GPUStereo/gpu_stereo_gui.py "$@"

