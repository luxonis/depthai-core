#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

source ~/emsdk/emsdk_env.sh

cd ../..

cmake --build build --target depthai-js --parallel $(($(nproc) * 75 / 100))
cp build/bindings/js/depthai-js.{d.ts,js} bindings/js/src/
cd -

if [ ! -d node_modules ]; then
  npm ci
fi

npm run build
