#!/bin/bash
set -e

FLAVOR=$1
BRANCH=$2
REGISTRY=$3
COMMIT_ID=$4

if [ -z "$FLAVOR" ] || [ -z "$BRANCH" ] || [ -z "$REGISTRY" ] || [ -z "$COMMIT_ID" ]; then
  echo "Usage: $0 <flavor> <branch> <registry> <commit_id>"
  exit 1
fi

REPO_NAME="depthai-core-hil"
TAG="${FLAVOR}_${COMMIT_ID}"
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
  --build-arg BRANCH="${BRANCH}"
fi

# Push the image
echo "🚀 Tagging and pushing image to ${REGISTRY}..."
docker tag "${IMAGE_NAME}" "${FULL_IMAGE_NAME}"
docker push "${FULL_IMAGE_NAME}"
echo "✅ Done."
