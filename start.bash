#!/bin/bash

source /opt/doly/rolling/setup.bash

# Make sure the sdk libraries can find the local needed libs already installed on Doly
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/.doly/libs/piper/lib"

if [ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]; then
    unset ROS_AUTOMATIC_DISCOVERY_RANGE
    # Start a zenohd in the background
    ros2 run rmw_zenoh_cpp rmw_zenohd &
    sleep 2 # Wait for zenohd to start
fi

exec ros2 launch doly_hw_interfaces hardware_interfaces_bringup.launch.xml --show-all-subprocesses-output
