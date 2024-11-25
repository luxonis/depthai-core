#!/bin/bash

echo "/home/$USER/hil_framework/lib_testbed/tools" >> $GITHUB_PATH
echo "PYTHONPATH="$PYTHONPATH:/home/$USER/hil_framework"" >> $GITHUB_ENV
echo "HIL_FRAMEWORK_PATH="/home/$USER/hil_framework"" >> $GITHUB_ENV
