#!/bin/bash
set -e

# Give Xvfb a moment to initialize
sleep 2

# Activate Python environment
source /workspace/venv/bin/activate

# Run tests for the passed target (e.g., rvc2 or rvc4) and forward any options
cd /workspace/tests
echo "Running tests with args: $@"
test_target=$1
shift
python3 -u run_tests.py "--$test_target" "$@"
