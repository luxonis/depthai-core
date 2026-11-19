#!/bin/bash

cd /data

systemctl stop depthai_gate
source env_set.sh
gdbserver localhost:2000 ./depthai-device-rvc4
