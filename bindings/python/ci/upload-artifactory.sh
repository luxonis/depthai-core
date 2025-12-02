#!/bin/bash

# Set PATH_PREFIX based on mode
if [ "$1" == "--snapshot" ]; then
    export PATH_PREFIX=luxonis-python-snapshot-local/depthai
else if [ "$1" == "--release" ]; then
    export PATH_PREFIX=luxonis-python-release-local/depthai
fi else
    echo "Error: Unknown option $1"
    echo "Usage: $0 [--snapshot | --release]"
    exit 1
fi

# Download jfrog CLI
curl -fL https://getcli.jfrog.io | sh

cd wheelhouse/audited/ || exit 1

../../jfrog config add --artifactory-url=$ARTIFACTORY_URL --user=$ARTIFACTORY_USER --password=$ARTIFACTORY_PASS
../../jfrog rt u "*" "$PATH_PREFIX/"
