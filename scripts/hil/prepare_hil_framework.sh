#!/bin/bash

echo "/home/$USER/hil_framework/lib_testbed/tools" >> $GITHUB_PATH
echo "PYTHONPATH="$PYTHONPATH:/home/$USER/hil_framework"" >> $GITHUB_ENV
echo "HIL_FRAMEWORK_PATH="/home/$USER/hil_framework"" >> $GITHUB_ENV

pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && git pull && git submodule update --init --recursive > /dev/null 2>&1 && popd > /dev/null 2>&1
pushd /home/$USER/hil_framework/ > /dev/null 2>&1 && pip install -r requirements.txt  > /dev/null 2>&1 && popd > /dev/null 2>&1