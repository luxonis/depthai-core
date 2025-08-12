#!/bin/sh
# $1 - path to the wheel
# $2 - path to the build directory
set -ex
EXTRA_DLL_PATH=$(find $2 -name "temp.win*" -type d)/Release/Release
echo "delvewheel extra dll path: $EXTRA_DLL_PATH"
delvewheel.exe repair $1 --add-path $EXTRA_DLL_PATH -w wheelhouse/audited