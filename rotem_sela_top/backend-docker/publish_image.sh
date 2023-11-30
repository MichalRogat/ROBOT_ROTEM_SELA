#!/bin/bash

. configuration.env

PROJ=$(pwd)

set -x

echo "Tag the image on the repository"
docker tag ${SERVICE_NAME}:${IMAGE_VERSION} artinbitslab/${SERVICE_NAME}:${IMAGE_VERSION}

echo "Push the image on the repository"
docker push artinbitslab/${SERVICE_NAME}:${IMAGE_VERSION}

echo "publish success"
