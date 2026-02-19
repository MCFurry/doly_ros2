#!/bin/bash

source /opt/dolly/rolling/setup.bash
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Start a zenohd in the background
ros2 run rmw_zenoh_cpp rmw_zenohd &
sleep 2 # Wait for zenohd to start

exec ros2 run drive_interface drive_interface_node --show-all-subprocesses-output
