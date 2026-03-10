#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${HOLOLINK_REPO_URL:-https://github.com/nvidia-holoscan/holoscan-sensor-bridge.git}"
REPO_REF="${HOLOLINK_REPO_REF:-2.5.0}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

usage() {
    cat <<EOF
Usage: $0 [--ref <tag-or-commit>] [--repo <git-url>] [--python <python-bin>]

Build and install hololink emulator Python bindings into the active environment.
Defaults:
  --repo   ${REPO_URL}
  --ref    ${REPO_REF}
  --python ${PYTHON_BIN}

Examples:
  $0
  $0 --ref main
  $0 --ref 6930609c4ce264ec7e2936dd1f5813323fccb08e
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ref)
            REPO_REF="$2"
            shift 2
            ;;
        --repo)
            REPO_URL="$2"
            shift 2
            ;;
        --python)
            PYTHON_BIN="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if ! command -v git >/dev/null 2>&1; then
    echo "Missing dependency: git" >&2
    exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
    echo "Missing dependency: cmake" >&2
    exit 1
fi
if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "Python interpreter not found: ${PYTHON_BIN}" >&2
    exit 1
fi

WORKDIR="$(mktemp -d)"
cleanup() {
    rm -rf "${WORKDIR}"
}
trap cleanup EXIT

SRC_DIR="${WORKDIR}/holoscan-sensor-bridge"
BUILD_DIR="${WORKDIR}/build"
PKG_DIR="${SRC_DIR}/src/hololink/emulation/python"
PKG_EMU_DIR="${PKG_DIR}/hololink/emulation"

echo "[1/5] Cloning ${REPO_URL} @ ${REPO_REF}"
if ! git clone --depth 1 --branch "${REPO_REF}" "${REPO_URL}" "${SRC_DIR}" 2>/dev/null; then
    git clone "${REPO_URL}" "${SRC_DIR}"
    (cd "${SRC_DIR}" && git checkout "${REPO_REF}")
fi

echo "[2/5] Configuring emulator-only build"
cmake -S "${SRC_DIR}/src/hololink/emulation" -B "${BUILD_DIR}" -DHSB_EMULATOR_BUILD_PYTHON=ON

JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
echo "[3/5] Building _emulation module with ${JOBS} job(s)"
cmake --build "${BUILD_DIR}" --target _emulation -j"${JOBS}"

EMULATION_SO="$(find "${BUILD_DIR}" -type f -name "_emulation*.so" | head -n1)"
if [[ -z "${EMULATION_SO}" ]]; then
    echo "Build succeeded but _emulation shared library was not found." >&2
    exit 1
fi

echo "[4/5] Installing hololink emulator package into active Python environment"
cp "${EMULATION_SO}" "${PKG_EMU_DIR}/"
"${PYTHON_BIN}" -m pip install "${PKG_DIR}"

echo "[5/5] Done"
echo "Installed hololink from ${REPO_URL} @ ${REPO_REF}"
echo "Note: LinuxDataPlane requires CAP_NET_RAW at runtime."
