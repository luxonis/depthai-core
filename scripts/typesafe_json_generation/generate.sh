#!/bin/bash

set -e
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1

folder_name="nn_archive/v1"
namespace="dai::nn_archive::v1"

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
main_dir=json_types/$folder_name
mkdir -p $main_dir
cd $main_dir
npx quicktype \
  --namespace "${namespace}" \
  --include-location global-include \
  --member-style camel-case \
  --lang c++ \
  --out Config.cpp \
  --src-lang schema \
  --code-format with-struct \
  --no-boost \
  --source-style multi-source \
  ../../../luxonis_ml_json_schema.json
cd ../../..
headers_dir=../../include/depthai/$folder_name
rm -rf $headers_dir
mkdir -p $headers_dir
cp -r $main_dir/* $headers_dir/
private_dir=../../src/$folder_name
rm -rf $private_dir
mkdir -p $private_dir
for private_file in \
  Generators.hpp \
  helper.hpp
do
  mv $headers_dir/$private_file $private_dir/
done

# fix private files
perl -0777 -i -pe 's/\#ifndef NLOHMANN_OPT_HELPER.*\#endif//s' $private_dir/helper.hpp
perl -0777 -i -pe 's|\#include \"|\#include \"depthai/nn_archive/v1/|g' $private_dir/Generators.hpp
perl -0777 -i -pe 's|\#include \"depthai/nn_archive/v1/helper.hpp\"|\#include \"helper.hpp\"|g' $private_dir/Generators.hpp

# cleanup public files
grep -rl 'To parse this JSON data' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\/\/  To parse this JSON data.*\#pragma once/\#pragma once/s'
grep -rl '#include "helper.hpp"' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\#include "helper.hpp"\n//s'
grep -rl '#include <nlohmann/json.hpp>' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/\#include <nlohmann\/json.hpp>\n//s'
grep -rl 'using nlohmann::json;' $headers_dir | xargs -n 1 perl -0777 -i -pe 's/ *using nlohmann::json;\n//s'

# add include <string>
grep -lZ std::string $headers_dir/*.hpp | xargs -r0 grep -L 'include <string>' | xargs -n 1 perl -0777 -i -pe 's/#pragma once\n\n/#pragma once\n\n#include <string>\n/s'
# add include <vector>
grep -lZ std::vector $headers_dir/*.hpp | xargs -r0 grep -L 'include <vector>' | xargs -n 1 perl -0777 -i -pe 's/#pragma once\n\n/#pragma once\n\n#include <vector>\n/s'

