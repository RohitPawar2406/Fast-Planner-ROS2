#!/bin/bash

# Set the Docker image name
IMAGE_NAME="foxy-desktop"

docker run -it --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./uav_simulator/:/ros2_ws/src/uav_simulator \
    -v ./fast_planner/:/ros2_ws/src/fast_planner \
    $IMAGE_NAME \
    /bin/bash