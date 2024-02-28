#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

source ~/emsdk/emsdk_env.sh

cd ../..

cmake --build build --target depthai-js --parallel $(($(nproc) * 75 / 100))
# the whole library ? will probably have to write bindings by hand as in the python one
# emcc -lembind -o library.js -s MODULARIZE=1 -Wl,--whole-archive build/libdepthai-core.a -Wl,--no-whole-archive

# just a subset / wrapper:
# em++ main.cpp
