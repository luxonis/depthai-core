#!/bin/bash

# Get depthai version from input argument or print message
DEPTHAI_VERSION="$1"

RELEASE_URL="https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/"
SNAPSHOT_URL="https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/"

if [ -z "$DEPTHAI_VERSION" ] || [ "$DEPTHAI_VERSION" == "latest" ]; then
    echo "Using latest depthai"
    source /home/hil/.hil/bin/activate
else
    echo "Installing depthai==$DEPTHAI_VERSION (checking both release and snapshot artifactories)"
    rm -rf venv
    python3 -m venv venv
    source venv/bin/activate
    pip install --no-cache-dir --extra-index-url "$RELEASE_URL" --extra-index-url "$SNAPSHOT_URL" depthai=="$DEPTHAI_VERSION"
fi

export DEPTHAI_PLATFORM=rvc4
export DISPLAY=:99

python tests/stability/stability_test_depthai.py
