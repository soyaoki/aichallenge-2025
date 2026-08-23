#!/bin/bash

# Single kart, no NPCs, camera on: the simplest scene for bringing up
# control_method:=openpilot. Use with `make simulator-openpilot`.
#
# wall-recovery is on: contact with a barrier otherwise freezes the kart for the
# rest of the run, which makes it impossible to tell a bad model from a wreck.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles 1 \
    --npcs 0 \
    --boosts 0 \
    --laps 6 \
    --timeout 600.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery on \
    --start-random off \
    --ranking off \
    --camera gpu \
    --lidar off \
    --imu on \
    --gnss on \
    --v2x off

# Cameraを使う場合 : --camera cpu or gpu
# LiDARを使う場合 : --lidar cpu or gpu
# GPUがない場合 -headlessを末尾に追加
