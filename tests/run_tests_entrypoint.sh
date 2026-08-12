#!/bin/bash
set -e

sleep 2
source /workspace/venv/bin/activate

cd /workspace/tests

while true; do
    echo "Running tests with args: $@"
    python3 run_tests.py "--$@" || true
    sleep 1
done