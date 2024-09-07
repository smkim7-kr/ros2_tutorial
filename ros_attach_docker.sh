#!/bin/bash

# Check if the container named "ros_humble" is running
container_status=$(docker inspect -f '{{.State.Running}}' ros_humble 2>/dev/null)

if [ "$container_status" == "true" ]; then
  echo "Attaching to the ros_humble container..."
  docker attach ros_humble
elif [ "$container_status" == "false" ]; then
  echo "The ros_humble container exists but is not running."
  echo "Starting the ros_humble container..."
  docker start ros_humble
  echo "Attaching to the ros_humble container..."
  docker attach ros_humble
else
  echo "No container named 'ros_humble' is found."
  echo "Please ensure the container is running or exists."
fi
