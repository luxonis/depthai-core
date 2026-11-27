#!/bin/bash
# Uploads the published build cache of the RVC4 firmware build (the output folder of ci/device-rvc4-build-cache.sh)
# to Artifact Keeper. The files of a commit go to device-rvc4-build-cache/commits/<commit>/. The vcpkg archives go
# to device-rvc4-build-cache/vcpkg/<key>/ and are not uploaded again when the server has that key.
#
# Usage: AK_URL=... AK_TOKEN=... ci/upload-ak-device-build-cache.sh <folder>
set -euo pipefail

: "${AK_URL:?AK_URL must be set}"
: "${AK_TOKEN:?AK_TOKEN must be set}"
folder=${1:?usage: ci/upload-ak-device-build-cache.sh <folder>}

AK_REPO="luxonis-depthai-helper-binaries"
PATH_PREFIX="device-rvc4-build-cache"
PUBLIC_URL="${DEPTHAI_DEVICE_BUILD_CACHE_BASE_URL:-https://artifacts.luxonis.com/artifactory/$AK_REPO/$PATH_PREFIX}"

bash "$(dirname "${BASH_SOURCE[0]}")/setup-ak-cli.sh"

export AK_NO_INPUT="${AK_NO_INPUT:-true}"

AK_INSTANCE="${AK_INSTANCE:-ci}"
AK_BASE_URL="${AK_URL%/}"
if ! ak instance list --format quiet | grep -Fxq "$AK_INSTANCE"; then
    ak instance add "$AK_INSTANCE" "$AK_BASE_URL"
fi

cd "$folder"
# The description of a commit goes last: a build of a user reads it first, and then the files that it names.
mapfile -t files < <(find vcpkg commits -type f ! -name build-cache.json | LC_ALL=C sort; find commits -type f -name build-cache.json | LC_ALL=C sort)
for file in "${files[@]}"; do
    if [[ "$file" == vcpkg/* ]] && curl -fsI -o /dev/null "$PUBLIC_URL/$file"; then
        echo "On the server already: $file"
        continue
    fi
    echo "Upload: $AK_REPO:$PATH_PREFIX/$file ($(du -sh "$file" | cut -f1))"
    ak artifact push "$AK_REPO" "$file" --path "$PATH_PREFIX/$file"
done
