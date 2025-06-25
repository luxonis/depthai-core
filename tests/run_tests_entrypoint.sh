#!/bin/bash
set -e

# Give Xvfb a moment to initialize
sleep 2

# Activate Python environment
source /workspace/venv/bin/activate

# Run your tests with passed arguments (e.g., rvc2 or rvc4)
cd /workspace/tests
echo "Running tests with args: $@"
python3 run_tests.py "--$@"
