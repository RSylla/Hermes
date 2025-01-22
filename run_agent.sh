#!/bin/bash

# This script runs a micro-ROS agent using Docker
# It mounts /dev and /dev/shm, uses host networking, and connects to a serial device

# Run the micro-ROS agent with Domain ID 7
docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyACM1 -v6 --domain-id 7
