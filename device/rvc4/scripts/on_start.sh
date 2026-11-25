#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"
. "$SCRIPT_DIR/gate_version.sh"
gate_version_current="$(gate_version_get)"
gate_version_limit="0.0.24"
if [ "$(gate_version_to_num $gate_version_current)" -lt "$(gate_version_to_num $gate_version_limit)" ]
then
    echo "$gate_version_current is smaller than $gate_version_limit"
    if systemctl status qmmf-server | grep running
    then
        factoryFile="/persist/factory/enabled"
        setupFile="/etc/agentconfd/.setup_complete"
        if [ ! -e "$factoryFile" ] && [ ! -e "$setupFile" ]; then
            echo "device is in setup mode, not stopping qmmf-server." >&2
        else
            echo stopping qmmf-server
            systemctl stop qmmf-server
        fi
    else
        echo qmmf-server not running, moving on
    fi
else
    echo "$gate_version_current is NOT smaller than $gate_version_limit. Skipping qmmf-server handling"
fi
