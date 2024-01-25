#!/usr/bin/bash
#/home/rogat/Projects/ROBOT_ROTEM_SELA/rotem_sela_top/backend-docker/rotem_sela_backend/camHubRestart.sh
# this script resets the USB hub in case cameras are stuck (or any other usb device)
# it needs uhubctl to be installed
# e.g. on ubuntu type
# sudo apt install uhubctl
# docs are in https://github.com/mvp/uhubctl
# the script is supposed to be ran as root (sudu) unless we fix permitions in udev

sudo uhubctl -a cycle -d 1 -l 1
sudo uhubctl -a cycle -d 1 -l 2
#sudo systemctl restart myservice
