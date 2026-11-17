#!/bin/bash

set -e

if [ $# -lt 1 ]
then
  echo "WRONG USAGE!!!"
  echo "CORRECT USAGE IS:"
  echo "./utility_scripts/parse_core_dump.sh /path/to/core_dump_0.0.1+cd0fc231d8c860acf11ab424109d7e42500626ae_20251105_225556.tar.gz"
  exit 1
else
  echo "Parsing core dump $1"
fi

DIR="$( dirname "${BASH_SOURCE[0]}" )"
TMP_DIR="core_dump_tmp_$(tr -dc 'a-zA-Z0-9' </dev/urandom | head -c 5)"
TMP_PATH="${DIR}/${TMP_DIR}"

mkdir "$TMP_PATH"
tar -xf $1 -C "$TMP_PATH"


"${DIR}/ex.sh" "/work/utility_scripts/container_based/debug_core_dump_helper.sh -batch /work/utility_scripts/$TMP_DIR/core.*"

rm -rf "$TMP_PATH"
