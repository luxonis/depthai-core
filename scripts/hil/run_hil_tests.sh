#!/bin/bash

# Check if the argument is provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <test_type> [--rvc2 | --rvc4]"
  echo "  test_type   Specify test flavor: vanilla, asan-ubsan, or tsan."
  echo "  --rvc2      Optional: Run tests with RVC2 configuration."
  echo "  --rvc4      Optional: Run tests with RVC4 configuration."
  exit 1
fi
# Get the test type from the argument
TEST_FLAVOR=$1
TEST_ARGS=$2

# Set up a Python virtual environment
rm -rf venv
python3 -m venv venv
source venv/bin/activate
export LC_ALL=en_US.UTF-8
locale

if [ "$TEST_FLAVOR" == "vanilla" ]; then
    echo $CMAKE_TOOLCHAIN_PATH
else
    export CMAKE_TOOLCHAIN_PATH=$PWD/cmake/toolchain/${FLAVOR}.cmake
fi

export PATH="$PATH:/home/hil/hil_framework/lib_testbed/tools"
export PYTHONPATH="$PYTHONPATH:/home/hil/hil_framework"
export HIL_FRAMEWORK_PATH="/home/hil/hil_framework"

# Install required Python packages
pip install numpy pytest pytest-html  > /dev/null 2>&1
pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && git pull && git submodule update --init --recursive > /dev/null 2>&1 && popd > /dev/null 2>&1
pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && pip install -r requirements.txt  > /dev/null 2>&1 && popd > /dev/null 2>&1

cmake -S . -B build -D CMAKE_BUILD_TYPE=Release -D HUNTER_ROOT=$HOME/.hun2_$TEST_FLAVOR -D DEPTHAI_VCPKG_INTERNAL_ONLY=OFF -D DEPTHAI_BUILD_EXAMPLES=ON -D DEPTHAI_BUILD_TESTS=ON -D DEPTHAI_TEST_EXAMPLES=ON -D DEPTHAI_BUILD_PYTHON=ON -D DEPTHAI_PYTHON_TEST_EXAMPLES=ON -D DEPTHAI_PYTHON_ENABLE_EXAMPLES=ON
cmake --build build --parallel 2 --config Release

export DISPLAY=:99
xdpyinfo -display $DISPLAY >/dev/null 2>&1 || (Xvfb $DISPLAY &)
cd tests
python3 run_tests.py $TEST_ARGS
