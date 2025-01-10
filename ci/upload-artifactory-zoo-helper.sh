#!/bin/bash

# Download jfrog cli
curl -fL https://getcli.jfrog.io | sh

# Set paths
export PATH_PREFIX=luxonis-depthai-helper-binaries/zoo_helper
export ZOO_HELPER_BINARY_LOCAL_PATH=build/zoo_helper

# Get git hash  
export ZOO_HELPER_GIT_HASH=$(git rev-parse HEAD)

# Upload binary to artifactory
./jfrog config add --artifactory-url=$ARTIFACTORY_URL --user=$ARTIFACTORY_USER --password=$ARTIFACTORY_PASS
./jfrog rt u "$ZOO_HELPER_BINARY_LOCAL_PATH" "$PATH_PREFIX/$ZOO_HELPER_GIT_HASH/"
