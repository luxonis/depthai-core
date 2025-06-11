#!/bin/bash

# Set up a Python virtual environment
export PATH="$PATH:/home/hil/hil_framework/lib_testbed/tools"
export PYTHONPATH="$PYTHONPATH:/home/hil/hil_framework"
export HIL_FRAMEWORK_PATH="/home/hil/hil_framework"

# Get depthai version from input argument or print message
if [ -z "$1" ] || [ "$1" == "latest" ]; then
    echo "Using latest depthai"
    source /home/hil/.hil/bin/activate
else
    DEPTHAI_VERSION="$1"
    rm -rf venv
    python3 -m venv venv
    source venv/bin/activate
    pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ depthai=="$DEPTHAI_VERSION"
fi

pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && pip install -r requirements.txt > /dev/null 2>&1 && popd > /dev/null 2>&1

export DEPTHAI_PLATFORM=rvc4
export DISPLAY=:99

python tests/stability/stability_test_depthai.py
