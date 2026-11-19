#!/bin/bash

cd /data

systemctl stop depthai_gate
source env_set.sh
./depthai-device-rvc4
