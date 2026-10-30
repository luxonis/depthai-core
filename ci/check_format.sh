#!/bin/bash
# Check clang-format compliance; fails on any resulting diff.
#
set -euo pipefail

if [[ $# -ge 1 ]]; then
    cmake --build "$1" --target clangformat
else
    git ls-files -z -- 'src/*.cpp' 'src/*.c' 'src/*.hpp' 'include/*.hpp' 'include/*.cpp' \
        | xargs -0 "${CLANG_FORMAT_BIN:-clang-format}" -style=file -i
fi

# Display diff
git --no-pager diff

# The following returns non-zero exit code if there is any difference
git diff-index --quiet HEAD --