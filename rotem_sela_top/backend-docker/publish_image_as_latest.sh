#!/bin/bash

. configuration.env

PROJ=$(pwd)

set -x

echo "Tag the image on the repository"
docker tag artinbitslab/${SERVICE_NAME}:${IMAGE_VERSION} artinbitslab/${SERVICE_NAME}:latest

echo "Push the image on the repository"
docker push artinbitslab/${SERVICE_NAME}:latest

echo "publish success"
