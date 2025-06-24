#!/bin/bash

# Download jfrog cli
curl -fL https://getcli.jfrog.io | sh

# Set paths
export PATH_PREFIX=luxonis-depthai-helper-binaries/zoo_helper/$ZOO_HELPER_PLATFORM
export ZOO_HELPER_BINARY_LOCAL_PATH=build/zoo_helper

# Get git hash  
git config --global --add safe.directory $(pwd)
export ZOO_HELPER_GIT_HASH=$(git rev-parse HEAD)

echo "----------------------------------------"
echo "PATH_PREFIX: $PATH_PREFIX"
echo "ZOO_HELPER_BINARY_LOCAL_PATH: $ZOO_HELPER_BINARY_LOCAL_PATH"
echo "ZOO_HELPER_GIT_HASH: $ZOO_HELPER_GIT_HASH"
echo "zoo_helper binary size: $(du -sh $ZOO_HELPER_BINARY_LOCAL_PATH)"
echo "zoo_helper download url: https://artifacts.luxonis.com/artifactory/$PATH_PREFIX/$ZOO_HELPER_GIT_HASH/zoo_helper"
echo "----------------------------------------"

# Upload binary to artifactory
./jfrog config add --artifactory-url=$ARTIFACTORY_URL --user=$ARTIFACTORY_USER --password=$ARTIFACTORY_PASS
./jfrog rt u "$ZOO_HELPER_BINARY_LOCAL_PATH" "$PATH_PREFIX/$ZOO_HELPER_GIT_HASH/"
