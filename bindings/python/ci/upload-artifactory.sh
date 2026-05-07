#!/bin/bash

# Set PATH_PREFIX based on mode
case "$1" in
    --snapshot)
        export PATH_PREFIX="luxonis-python-snapshot-local/depthai"
        ;;
    --release)
        export PATH_PREFIX="luxonis-python-release-local/depthai"
        ;;
    *)
        echo "Error: Unknown option $1"
        echo "Usage: $0 [--snapshot | --release]"
        exit 1
        ;;
esac

# Download JFrog CLI (v2)
curl -fL https://getcli.jfrog.io/v2-jf | sh

cd wheelhouse/audited/

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) JFROG="../../jf.exe" ;;
    *)                   JFROG="../../jf" ;;
esac

"$JFROG" config add luxonis \
    --artifactory-url="${ARTIFACTORY_URL}" \
    --user="${ARTIFACTORY_USER}" \
    --password="${ARTIFACTORY_PASS}" \
    --interactive=false \
    --overwrite=true

"$JFROG" rt u "*" "${PATH_PREFIX}/"
