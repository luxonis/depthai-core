#!/bin/bash
# Usage:
#   ci/device-rvc4-fwp-ak.sh version <repo dir>
#       Prints the version of that checkout: <first line of device/rvc4/version.txt>+<commit>.
#   ci/device-rvc4-fwp-ak.sh download <version> <target> <out dir>
#       Writes <out dir>/depthai-device-rvc4-fwp.tar.xz, -LICENSE, .spdx.json and .cdx.json.
#       Exit 0: downloaded and checked. Exit 2: not on the server, or a checksum does not match. Exit 1: other errors.
#   AK_URL=... AK_TOKEN=... ci/device-rvc4-fwp-ak.sh upload <version> <target> <dir with the four files>
#       Uploads each of the four files that the server does not have yet. Artifact Keeper makes the .sha256 files and
#       keeps the first upload of a snapshot version (409 Conflict), so when two runs upload the same version, the
#       files of the first one stay.
#
# DEPTHAI_DEVICE_RVC4_FWP_BASE_URL replaces the public base URL https://artifacts.luxonis.com/artifactory.
set -euo pipefail

AK_REPO="luxonis-rvc4-snapshot-local"
BASE_URL="${DEPTHAI_DEVICE_RVC4_FWP_BASE_URL:-https://artifacts.luxonis.com/artifactory}"
NAME="depthai-device-rvc4-fwp"
EXTENSIONS=(".tar.xz" "-LICENSE" ".spdx.json" ".cdx.json")
CURLE_FILE_COULDNT_READ_FILE=37
NO_HTTP_STATUS=000

usage() {
    sed -n '/^# Usage:/,/^# DEPTHAI/p' "$0" >&2
    exit 1
}

remote_names() {
    case "$2" in
        rvc4 | rvc4-*) ;;
        *) echo "Unknown target: $2" >&2; exit 1 ;;
    esac
    remote_dir="depthai-device-rvc4/$1"
    remote_stem="$NAME-$1${2#rvc4}"
}

fetch() {
    local status=0 code
    code=$(curl -sSL --retry 3 --connect-timeout 30 -o "$2" -w '%{http_code}' "$1") || status=$?
    if [ "$code" = 404 ] || [ "$status" = "$CURLE_FILE_COULDNT_READ_FILE" ]; then
        return 2
    fi
    if [ "$status" != 0 ] || { [ "$code" != 200 ] && [ "$code" != "$NO_HTTP_STATUS" ]; }; then
        echo "Download failed (curl exit $status, HTTP $code): $1" >&2
        return 1
    fi
}

sha256() {
    sha256sum "$1" | cut -d' ' -f1
}

cmd_version() {
    local base
    base=$(head -n 1 "$1/device/rvc4/version.txt")
    echo "${base//[[:space:]]/}+$(git -C "$1" rev-parse HEAD)"
}

cmd_download() {
    local version=$1 target=$2 out=$3 ext
    remote_names "$version" "$target"
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for ext in "${EXTENSIONS[@]}"; do
        local url="$BASE_URL/$AK_REPO/$remote_dir/$remote_stem$ext"
        fetch "$url.sha256" "$tmp/$NAME$ext.sha256" || return
        fetch "$url" "$tmp/$NAME$ext" || return
        if [ "$(sha256 "$tmp/$NAME$ext")" != "$(cat "$tmp/$NAME$ext.sha256")" ]; then
            echo "::warning::Checksum mismatch of $url"
            return 2
        fi
    done
    mkdir -p "$out"
    for ext in "${EXTENSIONS[@]}"; do
        mv "$tmp/$NAME$ext" "$out/$NAME$ext"
    done
    echo "Downloaded $remote_stem to $out"
}

cmd_upload() {
    : "${AK_URL:?AK_URL must be set}"
    : "${AK_TOKEN:?AK_TOKEN must be set}"
    local version=$1 target=$2 dir=$3 ext
    remote_names "$version" "$target"
    for ext in "${EXTENSIONS[@]}"; do
        [ -f "$dir/$NAME$ext" ] || { echo "Missing: $dir/$NAME$ext" >&2; exit 1; }
    done

    bash "$(dirname "${BASH_SOURCE[0]}")/setup-ak-cli.sh"
    export AK_NO_INPUT="${AK_NO_INPUT:-true}"
    local instance="${AK_INSTANCE:-ci}"
    if ! ak instance list --format quiet | grep -Fxq "$instance"; then
        ak instance add "$instance" "${AK_URL%/}"
    fi

    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for ext in "${EXTENSIONS[@]}"; do
        local url="$BASE_URL/$AK_REPO/$remote_dir/$remote_stem$ext"
        if fetch "$url.sha256" "$tmp/probe"; then
            echo "On the server already: $remote_dir/$remote_stem$ext"
            continue
        fi
        echo "Upload: $AK_REPO:$remote_dir/$remote_stem$ext ($(du -sh "$dir/$NAME$ext" | cut -f1))"
        if ! ak artifact push "$AK_REPO" "$dir/$NAME$ext" --path "$remote_dir/$remote_stem$ext"; then
            fetch "$url.sha256" "$tmp/probe" || { echo "Upload failed: $remote_dir/$remote_stem$ext" >&2; exit 1; }
            echo "Another run uploaded it first: $remote_dir/$remote_stem$ext"
        fi
    done
}

[ $# -ge 1 ] || usage
command=$1
shift
case "$command" in
    version) [ $# -eq 1 ] || usage; cmd_version "$@" ;;
    download) [ $# -eq 3 ] || usage; cmd_download "$@" ;;
    upload) [ $# -eq 3 ] || usage; cmd_upload "$@" ;;
    *) usage ;;
esac
