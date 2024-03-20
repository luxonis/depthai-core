#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

source ~/emsdk/emsdk_env.sh

if [ ! -d node_modules ]; then
  npm ci
fi

npx tsc --noEmit
npx esbuild src/index.ts --bundle --outfile=public/app.js
