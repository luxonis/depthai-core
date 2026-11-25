#!/bin/bash
# Puts the parts of the Qualcomm AI Runtime SDK (QAIRT, formerly SNPE) that the firmware build uses into a cache
# directory: the SNPE headers and libSNPE.so of the device OS target (22 MB of the SDK). The build scripts run it in
# the build container and mount the result into the sysroot (docker/container-env.sh).
#
# Usage:  fetch-qairt.sh <version> <cache dir>
# Result: <cache dir>/qairt/<version>/include/SNPE, lib/aarch64-oe-linux-gcc11.2/libSNPE.so, LICENSE.pdf, NOTICE.txt,
#         sdk.yaml
#
# QAIRT is Qualcomm software, and Luxonis may not redistribute it. This script takes it from one of:
#   DEPTHAI_DEVICE_QAIRT_ARCHIVE         the archive v<version>.zip that you downloaded from Qualcomm
#   DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes  the download server of Qualcomm. With this value you accept the license terms
#                                        of the SDK (LICENSE.pdf in the archive; a copy stays in the cache). The
#                                        script downloads only the parts in use (about 10 MB of the 1.1 GB archive).
set -euo pipefail

version=${1:?usage: fetch-qairt.sh <version> <cache dir>}
cache=${2:?usage: fetch-qairt.sh <version> <cache dir>}
libdir=aarch64-oe-linux-gcc11.2
dest=$cache/qairt/$version
# The path that messages show: the cache is a mount in the container
shown=${DEPTHAI_DEVICE_SDK_CACHE_LABEL:-$cache}/qairt/$version
url=https://softwarecenter.qualcomm.com/api/download/software/sdks/Qualcomm_AI_Runtime_Community/All/$version/v$version.zip

complete() { [ -f "$dest/include/SNPE/SNPE/SNPE.hpp" ] && [ -f "$dest/lib/$libdir/libSNPE.so" ]; }
if complete; then
    exit 0
fi

# The members of the archive that the firmware build uses, and the license and notice files that stay with them
members=("qairt/$version/include/SNPE/*" "qairt/$version/lib/$libdir/libSNPE.so")
notices=("qairt/$version/LICENSE.pdf" "qairt/$version/NOTICE.txt" "qairt/$version/sdk.yaml")

# Checksums of the versions that the firmware build was tested with: sha256 of the archive, and sha256 of the
# sha256sum list of the parts in use (parts_sha256 below)
case "$version" in
    2.32.6.250402)
        sha256=1fded6ecc74db7bc1c25e15aa8b7972e1feec0f70693fc5463f52015a492547b
        parts=840925cda623430caf8caf985b64e4314286b7fc7efb3061c160debae1becb31 ;;
    *) sha256=""; parts="" ;;
esac
parts_sha256() { (cd "$1" && find include/SNPE "lib/$libdir/libSNPE.so" -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -d' ' -f1); }

archive=${DEPTHAI_DEVICE_QAIRT_ARCHIVE:-}
downloaded=""
if [ -z "$archive" ]; then
    if [ "${DEPTHAI_ACCEPT_QUALCOMM_LICENSE:-}" != yes ]; then
        cat >&2 <<MESSAGE

QAIRT (Qualcomm AI Runtime SDK) $version is necessary for the firmware build: its headers and libSNPE.so.
It is not in the build image and not in the cache ($shown).
QAIRT is Qualcomm software under the license terms of Qualcomm (LICENSE.pdf in its archive). Luxonis may not
redistribute it.

To let the build download the parts in use from Qualcomm one time (about 10 MB), accept these terms:
    DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes
The download address is $url

Or download the archive yourself and give its path:
    DEPTHAI_DEVICE_QAIRT_ARCHIVE=/path/to/v$version.zip

MESSAGE
        exit 3
    fi
fi

mkdir -p "$cache/qairt"
tmp=$(mktemp -d "$cache/qairt/.$version.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
src=$tmp/qairt/$version

# The download server answers range requests, so the parts in use come without the rest of the 1.1 GB archive.
if [ -z "$archive" ]; then
    echo "Downloading the parts in use of QAIRT $version from Qualcomm: $url"
    if ! python3 "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/zip-fetch.py" "$url" "$tmp" "${members[@]}" "${notices[@]}"; then
        rm -rf "$src"
        mkdir -p "$cache/downloads"
        archive=$cache/downloads/v$version.zip
        echo "That download did not work. Downloading the complete archive (1.1 GB, no progress output) ..."
        curl -fsSL --retry 3 -C - -o "$archive.partial" "$url"
        mv "$archive.partial" "$archive"
        downloaded=yes
    fi
fi

if [ -n "$archive" ]; then
    [ -f "$archive" ] || { echo "error: QAIRT archive not found: $archive" >&2; exit 1; }
    if [ -n "$sha256" ] && ! echo "$sha256  $archive" | sha256sum -c - > /dev/null 2>&1; then
        echo "error: the archive is not the QAIRT $version archive that the firmware build was tested with (the sha256 is not $sha256)" >&2
        [ -z "$downloaded" ] || rm -f "$archive"
        exit 1
    fi
    echo "Extracting the SNPE headers and libSNPE.so of QAIRT $version ..."
    unzip -q "$archive" "${members[@]}" -d "$tmp"
    # An archive of another version can have other names for the license and notice files.
    unzip -q -o "$archive" "${notices[@]}" -d "$tmp" 2> /dev/null || true
fi

if [ ! -f "$src/include/SNPE/SNPE/SNPE.hpp" ] || [ ! -f "$src/lib/$libdir/libSNPE.so" ]; then
    echo "error: the archive has no qairt/$version/include/SNPE or no qairt/$version/lib/$libdir/libSNPE.so" >&2
    exit 1
fi
if [ -z "$parts" ]; then
    echo "warning: no checksum is known for QAIRT $version; its files are used without a check" >&2
elif [ "$(parts_sha256 "$src")" != "$parts" ]; then
    echo "error: the files are not the QAIRT $version files that the firmware build was tested with (the checksum of the parts in use is not $parts)" >&2
    exit 1
fi
# A copy that an earlier run did not complete is replaced. Another build can make the copy at the same time.
complete || rm -rf "$dest"
mv -T "$src" "$dest" 2> /dev/null || complete || { echo "error: cannot write $shown" >&2; exit 1; }
[ -z "$downloaded" ] || rm -f "$archive"
echo "QAIRT $version is in the cache: $shown (license terms of Qualcomm: LICENSE.pdf in that folder)"
