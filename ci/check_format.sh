#!/bin/bash

# Perform code format (cmake target)
cmake --build . --target clangformat

# Display diff
git --no-pager diff

# The following returns non-zero exit code if there is any difference
git diff-index --quiet HEAD --