#!/bin/bash

# Check if the argument is provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <pull_request> <test_type> [--rvc2 | --rvc4]"
  echo "  pull_request   Specify are we doing short tests (pull request) or long tests (dispatch/merge)"
  echo "  test_type   Specify test flavor: vanilla, asan-ubsan, or tsan."
  echo "  --rvc2      Optional: Run tests with RVC2 configuration."
  echo "  --rvc4      Optional: Run tests with RVC4 configuration."
  exit 1
fi
# Get the test type from the argument
PULL_REQUEST=$1
TEST_FLAVOR=$2
TEST_ARGS=$3

# Set up a Python virtual environment
rm -rf venv
python3.12 -m venv venv
source venv/bin/activate
export LC_ALL=en_US.UTF-8
locale

if [ "$TEST_FLAVOR" == "vanilla" ]; then
    echo $CMAKE_TOOLCHAIN_PATH
else
    export CMAKE_TOOLCHAIN_PATH=$PWD/cmake/toolchain/${TEST_FLAVOR}.cmake
fi

pip install numpy pytest pytest-html  > /dev/null 2>&1
export VCPKG_BINARY_SOURCES="files,/tmp/vcpkg_binaries,readwrite"
rm -rf build
CMAKE_ARGS=(
  -S . -B build
  -D CMAKE_BUILD_TYPE=Release
  -D DEPTHAI_VCPKG_INTERNAL_ONLY=OFF
  -D VCPKG_OVERLAY_TRIPLETS="$PWD/cmake/triplets/release"
  -D DEPTHAI_BUILD_TESTS=ON
  -D CMAKE_SHARED_LINKER_FLAGS="-Wl,-w"
)

if [ "$PULL_REQUEST" = "false" ]; then
  CMAKE_ARGS+=(
    -D DEPTHAI_BUILD_EXAMPLES=ON
    -D DEPTHAI_TEST_EXAMPLES=ON
    -D DEPTHAI_BUILD_PYTHON=ON
    -D DEPTHAI_PYTHON_TEST_EXAMPLES=ON
    -D DEPTHAI_PYTHON_ENABLE_EXAMPLES=ON
  )
fi

echo "Running:"
printf 'cmake'; printf ' %q' "${CMAKE_ARGS[@]}"; echo

cmake "${CMAKE_ARGS[@]}"

cmake --build build --parallel 3 --config Release \
  || { echo "Build failed"; exit 3; }

cd tests
python3 run_tests.py $TEST_ARGS
