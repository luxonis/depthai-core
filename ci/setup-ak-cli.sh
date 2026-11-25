#!/bin/bash
# Installs the Artifact Keeper CLI (ak) when it is not on PATH yet.
set -euo pipefail

AK_CLI_VERSION="v1.12.0"
INSTALLER_URL="https://raw.githubusercontent.com/artifact-keeper/artifact-keeper-cli/$AK_CLI_VERSION/install.sh"
ATTEMPTS=3

if command -v ak >/dev/null 2>&1; then
    exit 0
fi
for attempt in $(seq "$ATTEMPTS"); do
    if curl -fsSL --connect-timeout 30 --max-time 60 "$INSTALLER_URL" | sh -s -- --version "$AK_CLI_VERSION"; then
        exit 0
    fi
    if [ "$attempt" -lt "$ATTEMPTS" ]; then
        echo "Install of the Artifact Keeper CLI failed (attempt $attempt of $ATTEMPTS), next attempt in 10 s" >&2
        sleep 10
    fi
done
echo "Error: could not install the Artifact Keeper CLI $AK_CLI_VERSION" >&2
exit 1
