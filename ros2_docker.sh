#!/bin/bash

# Enable access to X11 display
xhost +local:docker

# Run the Docker container with X11 display forwarding and optional GPU support
docker run -it \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  --volume="/home/smkim/.bashrc:/root/.bashrc:rw" \
  --volume="/home/smkim/ros2_tutorials/robot_ws:/robot_ws" \
  --volume="/home/smkim/ros2_tutorials/dev_ws:/dev_ws" \
  --privileged \
  --name ros_humble2 \
  osrf/ros:humble-desktop

# Disable access to X11 display after running the container
xhost -local:docker
