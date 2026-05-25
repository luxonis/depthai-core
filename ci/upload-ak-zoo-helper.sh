#!/bin/bash
set -euo pipefail

: "${AK_URL:?AK_URL must be set}"
: "${AK_TOKEN:?AK_TOKEN must be set}"
: "${ZOO_HELPER_PLATFORM:?ZOO_HELPER_PLATFORM must be set}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

AK_REPO="luxonis-depthai-helper-binaries"

# Set paths
export PATH_PREFIX="zoo_helper/$ZOO_HELPER_PLATFORM"
export ZOO_HELPER_BINARY_LOCAL_PATH="build/zoo_helper"

# Get git hash
git config --global --add safe.directory "$(pwd)"
export ZOO_HELPER_GIT_HASH
ZOO_HELPER_GIT_HASH="$(git rev-parse HEAD)"

UPLOAD_PATH="$PATH_PREFIX/$ZOO_HELPER_GIT_HASH/zoo_helper"

echo "----------------------------------------"
echo "AK_REPO: $AK_REPO"
echo "PATH_PREFIX: $PATH_PREFIX"
echo "ZOO_HELPER_BINARY_LOCAL_PATH: $ZOO_HELPER_BINARY_LOCAL_PATH"
echo "ZOO_HELPER_GIT_HASH: $ZOO_HELPER_GIT_HASH"
echo "zoo_helper binary size: $(du -sh "$ZOO_HELPER_BINARY_LOCAL_PATH")"
echo "zoo_helper upload path: $AK_REPO:$UPLOAD_PATH"
echo "----------------------------------------"

if ! command -v ak >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/artifact-keeper/artifact-keeper-cli/main/install.sh | sh
fi

export AK_NO_INPUT="${AK_NO_INPUT:-true}"

AK_INSTANCE="${AK_INSTANCE:-ci}"
AK_BASE_URL="${AK_URL%/}"
if ! ak instance list --format quiet | grep -Fxq "$AK_INSTANCE"; then
    ak instance add "$AK_INSTANCE" "$AK_BASE_URL"
fi

ak artifact push "$AK_REPO" "$ZOO_HELPER_BINARY_LOCAL_PATH" --path "$UPLOAD_PATH"
