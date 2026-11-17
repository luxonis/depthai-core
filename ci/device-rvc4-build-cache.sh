#!/bin/bash
# Makes the published build cache of the RVC4 firmware build for the commit of this checkout:
#   - the object files of depthai-core, of its Python bindings and of the firmware sources that include no header of
#     a Qualcomm SDK, as a compiler cache (ccache)
#   - the vcpkg dependencies, as the binary archives of vcpkg
# A build of a user gets them with device/rvc4/docker/sdk/fetch-build-cache.py and compiles only the rest
# (device/rvc4/README.md, "Build time and caches").
#
# Usage:  ci/device-rvc4-build-cache.sh <output folder>
# Output: the files with their paths on the artifact server (ci/upload-ak-device-build-cache.sh uploads them):
#           <output folder>/commits/<commit>/build-cache.json
#           <output folder>/commits/<commit>/depthai-device-rvc4-ccache.tar.xz
#           <output folder>/vcpkg/<key>/depthai-device-rvc4-vcpkg.tar
#
# The build must be the build of a user, or the object files do not fit: target rvc4, the public release image, the
# prebuilt private module, no debug data, the build time of the commit, no CI variable. It needs a new checkout and
# it starts with empty caches, so that each object file is compiled here and goes into the cache.
# The Qualcomm AI Runtime SDK must be available as for each build with the public image (DEPTHAI_ACCEPT_QUALCOMM_LICENSE
# or DEPTHAI_DEVICE_QAIRT_ARCHIVE). No object file that is compiled with a header of a Qualcomm SDK goes into the
# published cache: device/rvc4/docker/published-cache-objects.py selects the object files and checks them.
set -euo pipefail

out=${1:?usage: ci/device-rvc4-build-cache.sh <output folder>}
ROOT=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
commit=$(git -C "$ROOT" rev-parse HEAD)
epoch=$(git -C "$ROOT" log -1 --format=%ct)
if [ -e "$ROOT/device/rvc4/build_docker_arm64_rvc4" ]; then
    echo "error: $ROOT/device/rvc4/build_docker_arm64_rvc4 exists. The build cache needs a build from a new checkout." >&2
    exit 1
fi
mkdir -p "$out"
out=$(readlink -f "$out")
work=$(mktemp -d "${TMPDIR:-/tmp}/device-rvc4-build-cache.XXXXXX")
trap 'rm -rf "$work"' EXIT

# The settings of a build of a user. A CI variable changes the version that the Python bindings report, and the
# build then keeps its caches in the build folder.
unset CI DEPTHAI_DEVICE_DEBUG_INFO SOURCE_DATE_EPOCH DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR DEPTHAI_DEVICE_RVC4_VENDOR HEXAGON_SDK_DIR
export DEPTHAI_DEVICE_SDK_CACHE="$work/sdk-cache" DEPTHAI_DEVICE_BUILD_CACHE=0 DEPTHAI_DEVICE_BUILD_CACHE_OUT="$work/ccache"
# A compile job can use 4 GB of memory
if [ -z "${DEPTHAI_BUILD_JOBS:-}" ]; then
    memory_jobs=$(( $(awk '/^MemAvailable:/ {print int($2 / 1048576)}' /proc/meminfo) / 4 ))
    DEPTHAI_BUILD_JOBS=$(( memory_jobs < 1 ? 1 : (memory_jobs < $(nproc) ? memory_jobs : $(nproc)) ))
    export DEPTHAI_BUILD_JOBS
fi
echo "Build cache of commit $commit, $DEPTHAI_BUILD_JOBS compile jobs, work folder $work"
"$ROOT/device/rvc4/create_deployment_artifacts.sh" rvc4

build="$ROOT/device/rvc4/build_docker_arm64_rvc4"
entries=$(find "$work/ccache" -type f | wc -l)
if [ "$entries" -lt 100 ]; then
    echo "error: the compiler cache has only $entries files" >&2
    exit 1
fi
# The license texts of the code in the object files: the license file of the firmware package
cp "$build/RelWithDebInfo/depthai-device-rvc4-fwp-LICENSE" "$work/ccache/LICENSE"

ccache_path="commits/$commit/depthai-device-rvc4-ccache.tar.xz"
mkdir -p "$out/commits/$commit"
tar -C "$work/ccache" --sort=name --owner=0 --group=0 --numeric-owner --mtime="@$epoch" -cf - . | xz -T0 -6 > "$out/$ccache_path"

# The binary archives of vcpkg: <two hex digits>/<ABI hash>.zip. Their names are the key of the set, so commits with
# the same dependencies share one file on the server. The archives are compressed, so the tar file is not.
vcpkg_cache="$work/sdk-cache/vcpkg"
mapfile -t archives < <(cd "$vcpkg_cache" && find . -mindepth 2 -maxdepth 2 -type f -name '*.zip' -path './[0-9a-f][0-9a-f]/*' | LC_ALL=C sort)
if [ "${#archives[@]}" -lt 10 ]; then
    echo "error: the vcpkg binary cache has only ${#archives[@]} archives" >&2
    exit 1
fi
key=$(printf '%s\n' "${archives[@]}" | sha256sum | cut -d' ' -f1)
vcpkg_path="vcpkg/$key/depthai-device-rvc4-vcpkg.tar"
mkdir -p "$out/vcpkg/$key"
tar -C "$vcpkg_cache" --owner=0 --group=0 --numeric-owner --mtime="@$epoch" -cf "$out/$vcpkg_path" "${archives[@]}"

python3 - "$out" "$commit" "$ccache_path" "$entries" "$key" "$vcpkg_path" "${#archives[@]}" <<'PY'
import hashlib, json, os, sys
out, commit, ccache_path, entries, key, vcpkg_path, archives = sys.argv[1:]
def describe(path):
    digest = hashlib.sha256()
    with open(os.path.join(out, path), "rb") as source:
        for block in iter(lambda: source.read(1 << 20), b""):
            digest.update(block)
    return {"path": path, "sha256": digest.hexdigest(), "size": os.path.getsize(os.path.join(out, path))}
manifest = {
    "format": 1,
    "commit": commit,
    "target": "rvc4",
    "ccache": {**describe(ccache_path), "files": int(entries)},
    "vcpkg": {"key": key, **describe(vcpkg_path), "archives": int(archives)},
}
with open(os.path.join(out, "commits", commit, "build-cache.json"), "w") as target:
    json.dump(manifest, target, indent=2)
    target.write("\n")
print(json.dumps(manifest, indent=2))
PY
