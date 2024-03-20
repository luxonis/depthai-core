#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

source ~/emsdk/emsdk_env.sh 

cd ../..

# these additional flags are needed so hunter doesn't fail with:
# [hunter ** FATAL ERROR **] ABI not detected for C compiler
# They shouldn't be needed ...
emcmake cmake -S. -Bbuild -DCMAKE_C_ABI_COMPILED=ON -DCMAKE_CXX_ABI_COMPILED=ON -DCMAKE_CROSSCOMPILING=ON

