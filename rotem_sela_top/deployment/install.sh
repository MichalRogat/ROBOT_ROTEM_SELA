#!/bin/bash

root=$(pwd)/../

echo "PREPARE SYSTEM"
apt update
apt install -y docker.io docker-compose gnupg2 pass

echo "SETUP FILES"
mkdir -p /opt/rotem_sela

cp ${root}/dockers/docker-compose.yml /opt/rotem_sela/

chmod aug+rw -R /opt/rotem_sela/

echo "CREATE ROTEM_SELA SERVICE"
cp ${root}/dockers/rotem_sela.service /etc/systemd/system/rotem_sela.service
systemctl enable rotem_sela
systemctl start rotem_sela
