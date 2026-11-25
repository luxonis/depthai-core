# Sourced by create_deployment_artifacts.sh and start_dev_shell.sh: the build image and the docker run arguments that
# both use (the environment variables are described in create_deployment_artifacts.sh).
# In: SOURCE_DIR (device/rvc4), CORE_DIR (the depthai-core checkout), DEPTHAI_DEVICE_SDK_IMAGE_KIND (release or dev:
# the public image to use when DEPTHAI_DEVICE_SDK_IMAGE is not set; release also means the build of the release
# firmware, target rvc4). Out: DEPTHAI_DEVICE_SDK_IMAGE, CONTAINER_ARGS, DEPTHAI_DEVICE_DEBUG_INFO.

# The default is the image that depthai-core pins (DEPTHAI_DEVICE_RVC4_SDK_IMAGE in
# cmake/Depthai/DepthaiDeviceRVC4Config.cmake). The release image has no sanitizer libraries. The dev image adds them.
if [ -z "${DEPTHAI_DEVICE_SDK_IMAGE:-}" ]; then
    DEPTHAI_DEVICE_SDK_IMAGE=$("$SOURCE_DIR/docker/sdk-images.sh" pinned "${DEPTHAI_DEVICE_SDK_IMAGE_KIND:-release}") || exit 1
fi
echo "Build image: $DEPTHAI_DEVICE_SDK_IMAGE"

# Vendor module: the sources only when DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR is set, else its prebuilt archive
# (DEPTHAI_DEVICE_RVC4_VENDOR, or the download of the version that depthai-core pins), as in CMakeLists.txt.
VENDOR_SOURCE_DIR="${DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR:-}"
CONTAINER_ARGS=()

# Caches on the host that all checkouts share: the compiler cache, the binary cache of the vcpkg dependencies and the
# QAIRT parts in use. With them a new clone or a new worktree builds from the results of the builds before it. CI jobs
# use them too: a job on a new machine (a hosted runner) starts with empty caches and takes the published build cache
# (below). For a cold build, set DEPTHAI_DEVICE_SDK_CACHE to a new folder and DEPTHAI_DEVICE_BUILD_CACHE=0.
SDK_CACHE="${DEPTHAI_DEVICE_SDK_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/depthai-device-sdk}"
mkdir -p "$SDK_CACHE"
SDK_CACHE=$(readlink -f "$SDK_CACHE")
mkdir -p "$SDK_CACHE/ccache" "$SDK_CACHE/vcpkg"
# CCACHE_COMPILERCHECK=content: the compiler is known by its content and not by its file time, so that a cache
# entry fits each copy of the build image (the published build cache is made in another copy).
CONTAINER_ARGS+=(
    -v "$SDK_CACHE/ccache":/sdk-cache/ccache -e CCACHE_DIR=/sdk-cache/ccache -e CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-10G}"
    -e CCACHE_COMPILERCHECK=content
    -v "$SDK_CACHE/vcpkg":/sdk-cache/vcpkg -e VCPKG_DEFAULT_BINARY_CACHE=/sdk-cache/vcpkg
)

# The build time that the firmware reports is the time of the commit (SOURCE_DATE_EPOCH: CMake takes it for its time
# stamps). Two builds of one commit then have the same generated version header, and the compiler cache of one build
# fits the other.
SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "$CORE_DIR" log -1 --format=%ct 2>/dev/null)}"
if [ -n "$SOURCE_DATE_EPOCH" ]; then
    CONTAINER_ARGS+=(-e SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH")
fi

# The release firmware of a public build (target rvc4 with the prebuilt private module) has no debug data by default.
# The firmware package is the same with and without it, and the build is faster and can use the published build
# cache. A build with the sources of the private module keeps the debug data: its debug file is for crash reports.
if [ "${DEPTHAI_DEVICE_SDK_IMAGE_KIND:-release}" = release ] && [ -z "$VENDOR_SOURCE_DIR" ]; then
    DEPTHAI_DEVICE_DEBUG_INFO="${DEPTHAI_DEVICE_DEBUG_INFO:-0}"
fi
case "${DEPTHAI_DEVICE_DEBUG_INFO:-1}" in 0|[Oo][Ff][Ff]|[Ff][Aa][Ll][Ss][Ee]|[Nn][Oo]) BUILD_HAS_DEBUG_INFO=no ;; *) BUILD_HAS_DEBUG_INFO=yes ;; esac
if [ -n "${DEPTHAI_DEVICE_DEBUG_INFO:-}" ]; then
    CONTAINER_ARGS+=(-e DEPTHAI_DEVICE_DEBUG_INFO="$DEPTHAI_DEVICE_DEBUG_INFO")
fi

# Published build cache: for each commit of the main branches a CI job publishes object files (a compiler cache,
# ccache) and the vcpkg dependencies (binary archives). The object files are those of depthai-core, of its Python
# bindings and of the firmware sources that include no header of a Qualcomm SDK: almost all of a first build.
# docker/sdk/fetch-build-cache.py gets them for the commit of this checkout, or for the nearest parent commit that
# has them, and ccache reads the object files as a second storage. A source that you changed is compiled, because
# ccache takes an object file only when the source and all its headers are the same. The published object files are
# those of a build without debug data and without the sources of the private module.
# DEPTHAI_DEVICE_BUILD_CACHE=0 turns this off.
if [ "${DEPTHAI_DEVICE_BUILD_CACHE:-1}" != 0 ] && [ "$BUILD_HAS_DEBUG_INFO" = no ] && [ -z "$VENDOR_SOURCE_DIR" ] \
    && [ "${DEPTHAI_DEVICE_SDK_IMAGE_KIND:-release}" = release ]; then
    BUILD_CACHE_URL="https://artifacts.luxonis.com/artifactory/luxonis-depthai-helper-binaries/device-rvc4-build-cache"
    BUILD_CACHE_URL="${DEPTHAI_DEVICE_BUILD_CACHE_BASE_URL:-$BUILD_CACHE_URL}"
    BUILD_CACHE_DIR="$SDK_CACHE/build-cache"
    # No mapfile: the bash of macOS (3.2) does not have it
    # shellcheck disable=SC2207 # commit hashes have no spaces
    BUILD_CACHE_COMMITS=($(git -C "$CORE_DIR" rev-list --first-parent -n 11 HEAD 2>/dev/null))
    # No container start when the disk has the answer: the cache of the newest commit, with a note for each newer
    # commit that the server has none (the note is good for one day).
    BUILD_CACHE_COMMIT=""
    BUILD_CACHE_ASK=""
    for commit in "${BUILD_CACHE_COMMITS[@]}"; do
        if [ -f "$BUILD_CACHE_DIR/commits/$commit/build-cache.json" ]; then BUILD_CACHE_COMMIT=$commit; break; fi
        if [ -z "$(find "$BUILD_CACHE_DIR/missing/$commit" -mmin -1440 2>/dev/null)" ]; then BUILD_CACHE_ASK=yes; break; fi
    done
    if [ -n "$BUILD_CACHE_ASK" ]; then
        mkdir -p "$BUILD_CACHE_DIR"
        # shellcheck disable=SC2086 # extra arguments are split at spaces on purpose
        BUILD_CACHE_COMMIT=$(docker run --rm --user="$(id -u):$(id -g)" -e HOME=/tmp ${DEPTHAI_DEVICE_DOCKER_RUN_ARGS:-} \
            -v "$BUILD_CACHE_DIR":/sdk-cache/build-cache -v "$SDK_CACHE/vcpkg":/sdk-cache/vcpkg \
            -v "$SOURCE_DIR/docker/sdk":/sdk-scripts:ro "$DEPTHAI_DEVICE_SDK_IMAGE" \
            python3 /sdk-scripts/fetch-build-cache.py "$BUILD_CACHE_URL" /sdk-cache/build-cache /sdk-cache/vcpkg \
            "${BUILD_CACHE_COMMITS[@]}") || BUILD_CACHE_COMMIT=""
    fi
    if [ -n "$BUILD_CACHE_COMMIT" ] && [ -d "$BUILD_CACHE_DIR/commits/$BUILD_CACHE_COMMIT/ccache" ]; then
        touch "$BUILD_CACHE_DIR/commits/$BUILD_CACHE_COMMIT"
        echo "Published build cache: commit ${BUILD_CACHE_COMMIT:0:12} ($BUILD_CACHE_DIR/commits/$BUILD_CACHE_COMMIT)"
        CONTAINER_ARGS+=(
            -v "$BUILD_CACHE_DIR/commits/$BUILD_CACHE_COMMIT/ccache":/sdk-cache/published:ro
            -e "CCACHE_SECONDARY_STORAGE=file:/sdk-cache/published|read-only"
        )
    else
        echo "Published build cache: none for this commit and its 10 parent commits"
    fi
fi

# A CI job that makes the published build cache gives the folder for the object files (ci/device-rvc4-build-cache.sh).
if [ -n "${DEPTHAI_DEVICE_BUILD_CACHE_OUT:-}" ]; then
    mkdir -p "$DEPTHAI_DEVICE_BUILD_CACHE_OUT"
    CONTAINER_ARGS+=(
        -v "$(readlink -f "$DEPTHAI_DEVICE_BUILD_CACHE_OUT")":/sdk-cache/build-cache-out
        -e DEPTHAI_DEVICE_BUILD_CACHE_OUT=/sdk-cache/build-cache-out
    )
fi

if [ -n "$VENDOR_SOURCE_DIR" ]; then
    VENDOR_SOURCE_DIR=$(readlink -f "$VENDOR_SOURCE_DIR")
    echo "Vendor module: sources $VENDOR_SOURCE_DIR (mounted at /depthai-device-rvc4-vendor)"
    CONTAINER_ARGS+=(-v "$VENDOR_SOURCE_DIR":/depthai-device-rvc4-vendor -e DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR=/depthai-device-rvc4-vendor
                     -e DEPTHAI_DEVICE_RVC4_VENDOR=)
elif [ -n "${DEPTHAI_DEVICE_RVC4_VENDOR:-}" ]; then
    if [ ! -f "$DEPTHAI_DEVICE_RVC4_VENDOR" ]; then
        echo "error: DEPTHAI_DEVICE_RVC4_VENDOR is not a file: '$DEPTHAI_DEVICE_RVC4_VENDOR'" >&2
        exit 1
    fi
    VENDOR_ARCHIVE=$(readlink -f "$DEPTHAI_DEVICE_RVC4_VENDOR")
    echo "Vendor module: local archive $VENDOR_ARCHIVE (mounted at /depthai-device-rvc4-vendor-archive/$(basename "$VENDOR_ARCHIVE"))"
    CONTAINER_ARGS+=(-v "$VENDOR_ARCHIVE":"/depthai-device-rvc4-vendor-archive/$(basename "$VENDOR_ARCHIVE")":ro
                     -e DEPTHAI_DEVICE_RVC4_VENDOR="/depthai-device-rvc4-vendor-archive/$(basename "$VENDOR_ARCHIVE")"
                     -e DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR= -e DEPTHAI_ACCEPT_VENDOR_EULA)
else
    echo "Vendor module: the prebuilt archive of DEPTHAI_DEVICE_RVC4_VENDOR_VERSION (the build downloads it)"
    CONTAINER_ARGS+=(-e DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR= -e DEPTHAI_DEVICE_RVC4_VENDOR= -e DEPTHAI_ACCEPT_VENDOR_EULA)
fi

if [ -n "${HEXAGON_SDK_DIR:-}" ]; then
    HEXAGON_SDK_DIR=$(readlink -f "$HEXAGON_SDK_DIR")
    echo "Hexagon SDK: $HEXAGON_SDK_DIR (mounted at /opt/hexagon-sdk)"
    CONTAINER_ARGS+=(-v "$HEXAGON_SDK_DIR":/opt/hexagon-sdk -e HEXAGON_SDK_ROOT=/opt/hexagon-sdk)
fi

# QAIRT (Qualcomm AI Runtime SDK): the build needs its headers and libSNPE.so in the sysroot. The Luxonis OS SDK image
# has them. The public image does not, because Luxonis may not redistribute QAIRT. For such an image the parts in use
# come from a cache on the host. docker/sdk/fetch-qairt.sh fills the cache: it downloads these parts from Qualcomm when
# DEPTHAI_ACCEPT_QUALCOMM_LICENSE=yes, or it takes them from the archive DEPTHAI_DEVICE_QAIRT_ARCHIVE.
QAIRT_VERSION=$(sed -n 's/^set(DEPTHAI_DEVICE_QAIRT_VERSION "\([^"]*\)".*/\1/p' "$SOURCE_DIR/CMakeLists.txt")
# Prints the sysroot of the image when the image has one (DEPTHAI_DEVICE_RVC4_SYSROOT) without this QAIRT version
QAIRT_SYSROOT=$(docker run --rm "$DEPTHAI_DEVICE_SDK_IMAGE" sh -c '
    [ -n "${DEPTHAI_DEVICE_RVC4_SYSROOT:-}" ] || exit 0
    [ -d "$DEPTHAI_DEVICE_RVC4_SYSROOT/opt/qcom/aistack/qairt/'"$QAIRT_VERSION"'/include/SNPE" ] || echo "$DEPTHAI_DEVICE_RVC4_SYSROOT"') \
    || { echo "error: cannot run the build image $DEPTHAI_DEVICE_SDK_IMAGE" >&2; exit 1; }
if [ -n "$QAIRT_SYSROOT" ]; then
    QAIRT_FETCH_ARGS=(-e DEPTHAI_ACCEPT_QUALCOMM_LICENSE -e DEPTHAI_DEVICE_SDK_CACHE_LABEL="$SDK_CACHE")
    if [ -n "${DEPTHAI_DEVICE_QAIRT_ARCHIVE:-}" ]; then
        QAIRT_FETCH_ARGS+=(-v "$(readlink -f "$DEPTHAI_DEVICE_QAIRT_ARCHIVE")":/qairt-archive.zip:ro -e DEPTHAI_DEVICE_QAIRT_ARCHIVE=/qairt-archive.zip)
    fi
    # shellcheck disable=SC2086 # extra arguments are split at spaces on purpose
    docker run --rm --user="$(id -u):$(id -g)" -e HOME=/tmp ${DEPTHAI_DEVICE_DOCKER_RUN_ARGS:-} "${QAIRT_FETCH_ARGS[@]}" \
        -v "$SDK_CACHE":/sdk-cache -v "$SOURCE_DIR/docker/sdk":/sdk-scripts:ro \
        "$DEPTHAI_DEVICE_SDK_IMAGE" /sdk-scripts/fetch-qairt.sh "$QAIRT_VERSION" /sdk-cache || exit 1
    QAIRT_DIR="$SDK_CACHE/qairt/$QAIRT_VERSION"
    echo "QAIRT $QAIRT_VERSION: $QAIRT_DIR (mounted into the sysroot)"
    # The firmware links -lSNPE from usr/lib, as in the OS sysroot. A link directory would put the QAIRT folder into
    # the run path of the firmware, and the device would then load that libSNPE.so and not the one of its OS.
    CONTAINER_ARGS+=(
        -v "$QAIRT_DIR":"$QAIRT_SYSROOT/opt/qcom/aistack/qairt/$QAIRT_VERSION":ro
        -v "$QAIRT_DIR/lib/aarch64-oe-linux-gcc11.2/libSNPE.so":"$QAIRT_SYSROOT/usr/lib/libSNPE.so":ro
    )
fi

# A git worktree keeps its metadata in the main checkout, outside the mounted tree. Mount that directory at the same
# path, read-only, so that the build can read the commit and the submodules.
GIT_COMMON_DIR=$(cd "$CORE_DIR" && readlink -f "$(git rev-parse --git-common-dir 2>/dev/null)" 2>/dev/null)
case "$GIT_COMMON_DIR" in
    ""|"$CORE_DIR"/*) ;;
    *) CONTAINER_ARGS+=(-v "$GIT_COMMON_DIR":"$GIT_COMMON_DIR":ro) ;;
esac

# The calling user in the container (passwd and shadow, and the group 'sudo' (27) for images that have sudo), the
# checkout at /work
# shellcheck disable=SC2206 # extra arguments are split at spaces on purpose
CONTAINER_ARGS+=(${DEPTHAI_DEVICE_DOCKER_RUN_ARGS:-})
CONTAINER_ARGS+=(
    -e DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL="${DEPTHAI_DEVICE_RVC4_VENDOR_BASE_URL:-}"
    -v /etc/passwd:/etc/passwd:ro
    -v /etc/shadow:/etc/shadow:ro
    -v "$CORE_DIR":/work
    -w /work/device/rvc4
    --user="$(id -u):$(id -g)"
    --group-add 27
)
