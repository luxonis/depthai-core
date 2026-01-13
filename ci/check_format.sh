#!/bin/bash
set -euo pipefail

# Perform code format (cmake target)
cmake --build "$1" --target clangformat

# Check for missing public API docs (non-zero exit fails CI)
python3 scripts/check_public_docs.py

# Display diff
git --no-pager diff

# The following returns non-zero exit code if there is any difference
git diff-index --quiet HEAD --