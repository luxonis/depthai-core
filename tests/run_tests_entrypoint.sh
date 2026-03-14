#!/bin/bash
set -e

# Usage: ./tests/run_tests_entrypoint.sh <target> [dcl_mode]
TARGET="${1:-}"
DCL_MODE="${2:-}"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 <rvc2|rvc4|rvc4usb|rvc4rgb|fsync> [ON_START|CONTINUOUS|OFF]"
  exit 2
fi

# Optional second argument controls startup AutoCalibration mode.
if [ -n "$DCL_MODE" ]; then
  case "$DCL_MODE" in
    ON_START|CONTINUOUS|OFF)
      export DEPTHAI_AUTOCALIBRATION="$DCL_MODE"
      ;;
    *)
      echo "Error: Invalid DCL_MODE '$DCL_MODE'. Allowed values: ON_START, CONTINUOUS, OFF." >&2
      exit 1
      ;;
  esac
fi

# Give Xvfb a moment to initialize
sleep 2

# Activate Python environment
source /workspace/venv/bin/activate

# Run your tests with passed arguments (e.g., rvc2 or rvc4)
cd /workspace/tests
echo "Running tests target: $TARGET"
python3 run_tests.py "--$TARGET"
