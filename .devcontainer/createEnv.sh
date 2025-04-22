#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DOCKER_USER=$(id -u):$(id -g)
echo "DOCKER_USER=$DOCKER_USER" > $SCRIPT_DIR/.env
