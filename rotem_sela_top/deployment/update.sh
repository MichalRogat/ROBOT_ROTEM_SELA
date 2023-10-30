#!/bin/bash

root=$(pwd)/../

echo "Stop dockers"
cd /opt/rotem_sela
docker-compose down
sync

echo "backup previous data"
tar cfz backup-$(date +%Y-%m-%d).tar.gz .

echo "Update config files"
cp ${root}/dockers/docker-compose.yml /opt/rotem_sela/

sync
chmod aug+rw -R /opt/rotem_sela/

echo "Pull changes"
docker-compose pull

echo "Restart dockers"
docker-compose up -d
