#!/bin/bash

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

awsim_extra_args=()
if [[ "${AWSIM_FORCE_VULKAN:-}" == "1" ]]; then
    awsim_extra_args+=("-force-vulkan")
    awsim_extra_args+=("-force-device-index")
    awsim_extra_args+=("${AWSIM_FORCE_DEVICE_INDEX:-0}")
fi

# One-vehicle, camera-enabled closed-loop scenario for Vision Pilot tuning.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 15 \
    --vehicles 1 \
    --npcs 0 \
    --boosts 0 \
    --laps unlimited \
    --timeout 10000000.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random off \
    --ranking off \
    --camera cpu \
    --lidar off \
    --imu on \
    --gnss on \
    --v2x off \
    "${awsim_extra_args[@]}"
