#!/bin/bash
set -e

FLAVOR=$1
BRANCH=$2
REGISTRY=$3
COMMIT_ID=$4
PARALLEL_JOBS=$5
PULL_REQUEST=$6
TAG=$7

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

# Check if image exists locally
if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
  echo "✅ Local image ${IMAGE_NAME} already exists. Skipping build."
else
  # Build the image
    echo "🔨 Building image ${IMAGE_NAME}..."
    docker build -t "${IMAGE_NAME}" -f tests/Dockerfile . \
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
