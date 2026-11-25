#!/bin/bash

ROOT_DIR=$(realpath $(dirname $0))
DSP_ARCH=v73
DSP_OUTPUT_BINARY_DIR=$(pwd)

if [ ! -z $1 ]; then
    DSP_ARCH=$1
fi

if [ ! -z $2 ]; then
    DSP_OUTPUT_BINARY_DIR=$2
fi

# Force the environment set
HEXAGON_SDK_ROOT_FOLDER=$HEXAGON_SDK_ROOT
HEXAGON_SDK_ROOT=

source $HEXAGON_SDK_ROOT_FOLDER/setup_sdk_env.source > /dev/null 2>&1

make -C $ROOT_DIR V=hexagon_Release_toolv87_$DSP_ARCH \
    BUILD_DIR=$DSP_OUTPUT_BINARY_DIR dlls
