#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

child_namespace="nn_archive_v1"
parent_namespace="dai"

if [ ! -d luxonis_ml ]; then
  git clone https://github.com/luxonis/luxonis-ml.git luxonis_ml
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
if [ ! -d node_modules ]; then
  npm ci
fi
python generate_json_schema.py > luxonis_ml_json_schema_broken.json
node fix_schema_definition.js luxonis_ml_json_schema_broken.json luxonis_ml_json_schema.json
rm -rf json_types
main_dir=json_types/$child_namespace
mkdir -p $main_dir
cd $main_dir
npx quicktype \
  --namespace "${parent_namespace}::${child_namespace}" \
  --include-location global-include \
  --member-style camel-case \
  --lang c++ \
  --out Config.cpp \
  --src-lang schema \
  --code-format with-struct \
  --no-boost \
  --source-style multi-source \
  ../../luxonis_ml_json_schema.json
cd ../..
headers_dir=../../include/depthai/$child_namespace
rm -rf $headers_dir
cp -r $main_dir $headers_dir
private_dir=../../src/$child_namespace
rm -rf $private_dir
mkdir $private_dir
for private_file in \
  Generators.hpp \
  helper.hpp
do
  mv $headers_dir/$private_file $private_dir/
done

# fix private files
perl -0777 -i -pe 's/\#ifndef NLOHMANN_OPT_HELPER.*\#endif//s' $private_dir/helper.hpp
perl -0777 -i -pe 's|\#include \"|\#include \"depthai/nn_archive_v1/|g' $private_dir/Generators.hpp
perl -0777 -i -pe 's|\#include \"depthai/nn_archive_v1/helper.hpp\"|\#include \"helper.hpp\"|g' $private_dir/Generators.hpp

# cleanup public files
grep -rl 'To parse this JSON data' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\/\/  To parse this JSON data.*\#pragma once/\#pragma once/s'
grep -rl '#include "helper.hpp"' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\#include "helper.hpp"\n//s'
grep -rl '#include <nlohmann/json.hpp>' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\#include <nlohmann\/json.hpp>\n//s'
grep -rl 'using nlohmann::json;' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/ *using nlohmann::json;\n//s'

