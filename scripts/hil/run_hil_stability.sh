#!/bin/bash

# Get depthai version from input argument or print message
if [ -z "$1" ] || [ "$1" == "latest" ]; then
    echo "Using latest depthai"
    source /home/hil/.hil/bin/activate
else
    DEPTHAI_VERSION="$1"
    rm -rf venv
    python3 -m venv venv
    source venv/bin/activate
    pip install --no-cache-dir --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ depthai=="$DEPTHAI_VERSION"
fi

export DEPTHAI_PLATFORM=rvc4
export DISPLAY=:99

python tests/stability/stability_test_depthai.py
