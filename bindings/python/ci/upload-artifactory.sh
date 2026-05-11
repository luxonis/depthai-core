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

cd wheelhouse/audited/ || exit 1

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
        curl -fLg -o jfrog.exe \
            "https://releases.jfrog.io/artifactory/jfrog-cli/v1/[RELEASE]/jfrog-cli-windows-amd64/jfrog.exe"
        JFROG="./jfrog.exe"
        ;;
    *)
        curl -fL https://getcli.jfrog.io | sh
        JFROG="./jfrog"
        ;;
esac

if [[ ! -f "$JFROG" ]]; then
    echo "Error: JFrog CLI download succeeded, but $JFROG was not found."
    exit 1
fi

echo "Detected JFrog binary: $JFROG"

"$JFROG" --version

"$JFROG" config add --artifactory-url="$ARTIFACTORY_URL" --user="$ARTIFACTORY_USER" --password="$ARTIFACTORY_PASS"
"$JFROG" rt u "*" "$PATH_PREFIX/"
