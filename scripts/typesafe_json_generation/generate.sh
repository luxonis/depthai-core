#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1


if [ ! -d luxonis_ml ]; then
  git clone https://github.com:luxonis/luxonis-ml.git luxonis_ml
  cd luxonis_ml
  git checkout dev
  cd ..
fi
if [ ! -d env ]; then
  python3 -m venv env
  source env/bin/activate
  pip install -r luxonis_ml/luxonis_ml/nn_archive/requirements.txt
else
  source env/bin/activate
fi
python generate_json_schema.py > luxonis_ml_json_schema_broken.json
node fix_schema_definition.js luxonis_ml_json_schema_broken.json luxonis_ml_json_schema.json
rm -rf json_types
mkdir -p json_types
cd json_types
npx quicktype \
  --namespace "dai::json_types" \
  --include-location global-include \
  --member-style camel-case \
  --lang c++ \
  --out NnArchiveConfig.cpp \
  --src-lang schema \
  --code-format with-struct \
  --no-boost \
  --source-style multi-source \
  ../luxonis_ml_json_schema.json
cd ..
rm -rf ../../src/json_types
cp -r json_types ../../src/

