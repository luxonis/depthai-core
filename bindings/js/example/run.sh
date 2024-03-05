#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

# This has to exist, if it doesn't please run ./bindings/js/configure.sh && ./bindings/js/build.sh first
cp ../../../build/bindings/js/depthai-js.wasm public/
./build.sh
cd public
echo Example is running...
echo Please navigate to http://localhost:8000/ in a browser and check the javascript console output.
echo Then modify index.html to suit your needs.
python3 -m http.server

