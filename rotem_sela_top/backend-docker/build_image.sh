#!/bin/bash

. configuration.env

PROJ=$(pwd)

set -x

echo "docker build image"
docker-compose build

echo "build success"
