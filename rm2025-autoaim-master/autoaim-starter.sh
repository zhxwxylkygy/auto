#!/bin/bash
 
 CONTAINER_NAME=vd

docker start ${CONTAINER_NAME}

sleep 5s

ssh root@vision.dev  "zsh /root/master/program/RM2025-autoaim/PIE_2025autoaim_watchdog.zsh"




