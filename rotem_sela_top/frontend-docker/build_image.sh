#!/bin/bash

. configuration.env

PROJ=$(pwd)

set -x

#git submodule update --init --recursive

echo "docker build image"
docker-compose build

echo "build success"
