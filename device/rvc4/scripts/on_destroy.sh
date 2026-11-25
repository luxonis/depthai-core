#!/bin/bash

echo "ON DESTROY CALLED"
SCRIPT_DIR="$(dirname "$0")"
. "$SCRIPT_DIR/gate_version.sh"
gate_version_current="$(gate_version_get)"
gate_version_limit="0.0.24"
if [ "$(gate_version_to_num $gate_version_current)" -lt "$(gate_version_to_num $gate_version_limit)" ]
then
    echo "$gate_version_current is smaller than $gate_version_limit"
    echo starting qmmf-server
    systemctl start qmmf-server
else
    echo "$gate_version_current is NOT smaller than $gate_version_limit. Skipping qmmf-server handling"
fi
