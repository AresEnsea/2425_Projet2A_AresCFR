#!/bin/bash
ROS_WORKSPACE=/home/carpe-bleue/ros2_ws
cd $ROS_WORKSPACE

source /opt/ros/humble/setup.bash
source install/setup.bash
sudo chmod 666 /dev/serial0
ros2 launch serial_package bag_player_node.py


