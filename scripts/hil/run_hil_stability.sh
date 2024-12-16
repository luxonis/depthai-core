#!/bin/bash

# Set up a Python virtual environment
rm -rf venv
python3 -m venv venv
source venv/bin/activate
rm -rf build/
export PATH="$PATH:/home/hil/hil_framework/lib_testbed/tools"
export PYTHONPATH="$PYTHONPATH:/home/hil/hil_framework"
export HIL_FRAMEWORK_PATH="/home/hil/hil_framework"
source /home/hil/.SETUP_CONFIG_VARS

# Install required Python packages
pip install numpy pytest pytest-html  > /dev/null 2>&1
pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && pip install -r requirements.txt  > /dev/null 2>&1 && popd > /dev/null 2>&1

cmake -S . -B build -D CMAKE_BUILD_TYPE=Release -D HUNTER_ROOT=$HOME/.hun_vanilla -D DEPTHAI_BUILD_TESTS=ON
cmake --build build --parallel 8 --config Release --target stability_stress_test
cd build
../ci/stability_stress_test_combined.sh 86400
