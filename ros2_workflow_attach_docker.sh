choose#!/bin/bash

# Check if the container named "ros_humble_robots" is running
container_status=$(docker inspect -f '{{.State.Running}}' ros_humble_robots 2>/dev/null)

if [ "$container_status" == "true" ]; then
  echo "The 'ros_humble_robots' container is running."
  
  # Ask if the user wants to attach or exec into the container
  read -p "Do you want to (a) attach to the container's primary process or (e) exec into a new shell? (a/e): " choice
  
  if [ "$choice" == "a" ]; then
    echo "Attaching to the ros_humble_robots container..."
    docker attach ros_humble_robots
  elif [ "$choice" == "e" ]; then
    echo "Opening a new shell in the ros_humble_robots container..."
    docker exec -it ros_humble_robots /bin/bash
  else
    echo "Invalid choice. Please choose 'a' to attach or 'e' to exec."
  fi

elif [ "$container_status" == "false" ]; then
  echo "The 'ros_humble_robots' container exists but is not running."
  echo "Starting the ros_humble_robots container..."
  docker start ros_humble_robots
  echo "Opening a new shell in the ros_humble_robots container..."
  docker exec -it ros_humble_robots /bin/bash
else
  echo "No container named 'ros_humble_robots' is found."
  echo "Please ensure the container is running or exists."
fi
