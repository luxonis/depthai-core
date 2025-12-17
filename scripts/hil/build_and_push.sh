#!/bin/bash
set -e

FLAVOR=$1
BRANCH=$2
REGISTRY=$3
COMMIT_ID=$4
PARALLEL_JOBS=$5
PULL_REQUEST=$6
TAG=$7
REFRESH_CACHE=${8:-true}   # optional

: "${PARALLEL_JOBS:=8}"  # Fallback to 8 if not set or passed
: "${PULL_REQUEST:="false"}"  # Fallback to false if not set or passed


if [ -z "$FLAVOR" ] || [ -z "$BRANCH" ] || [ -z "$REGISTRY" ] || [ -z "$COMMIT_ID" ] || [ -z "$PARALLEL_JOBS" ] || [ -z "$PULL_REQUEST" ] || [ -z "$TAG" ]; then
  echo "Usage: $0 <flavor> <branch> <registry> <commit_id> <number_of_cores> <is_pipeline_pull_request> <tag>"
  exit 1
fi

REPO_NAME="depthai-core-hil"

IMAGE_NAME="${REPO_NAME}:${TAG}"
FULL_IMAGE_NAME="${REGISTRY}/${IMAGE_NAME}"

echo "Checking for existing image: $FULL_IMAGE_NAME"

# Check if image exists remotely
if curl --silent --fail "http://${REGISTRY}/v2/${REPO_NAME}/manifests/${TAG}" \
    -H "Accept: application/vnd.docker.distribution.manifest.v2+json" > /dev/null; then
  echo "✅ Remote image ${FULL_IMAGE_NAME} already exists. Skipping build and push."
  exit 0
fi

# Ensure BuildKit + buildx are used
export DOCKER_BUILDKIT=1

BUILDER_NAME="hil-builder"
if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  echo "Creating buildx builder: ${BUILDER_NAME}"
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
else
  docker buildx use "${BUILDER_NAME}"
fi

# Cache scope: tie it to things that affect dependencies
CACHE_SCOPE="flavor-${FLAVOR}"
if [ "${PULL_REQUEST}" = "true" ]; then
  CACHE_SCOPE="${CACHE_SCOPE}_pr"
fi

# If you want to "refresh", write into a new cache directory
CACHE_ROOT="/tmp/buildkit-cache/depthai-core-hil"
if [ "${REFRESH_CACHE}" = "true" ]; then
  CACHE_DIR="${CACHE_ROOT}/${CACHE_SCOPE}-$(date +%Y%m%d%H%M%S)"
  echo "♻️ Refresh cache requested. Using new cache dir: ${CACHE_DIR}"
else
  CACHE_DIR="${CACHE_ROOT}/${CACHE_SCOPE}"
fi

sudo mkdir -p "${CACHE_DIR}"
sudo chown -R "$(id -u):$(id -g)" "${CACHE_ROOT}" || true

echo "🔨 Building + pushing with buildx. Cache dir: ${CACHE_DIR}"

# Check if image exists locally
if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
  echo "✅ Local image ${IMAGE_NAME} already exists. Skipping build."
else
  # Build the image
    echo "🔨 Building image ${IMAGE_NAME}..."
    docker buildx build -t "${IMAGE_NAME}" -f tests/Dockerfile . \
  --build-arg FLAVOR="${FLAVOR}" \
  --build-arg BRANCH="${BRANCH}" \
  --build-arg GIT_COMMIT="${COMMIT_ID}" \
  --build-arg PARALLEL_JOBS="${PARALLEL_JOBS}" \
  --build-arg PULL_REQUEST="${PULL_REQUEST}"
fi

# Push the image
echo "🚀 Tagging and pushing image to ${REGISTRY}..."
docker tag "${IMAGE_NAME}" "${FULL_IMAGE_NAME}"
docker push "${FULL_IMAGE_NAME}"
echo "✅ Done."
