#!/bin/bash
# Source ROS and workspace setup scripts
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/ros2_ws
# Start an interactive bash shell
exec "$@"
