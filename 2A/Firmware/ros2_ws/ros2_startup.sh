#!/bin/bash
ROS_WORKSPACE=/home/carpe-bleue/ros2_ws
cd $ROS_WORKSPACE
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serial_package launch_robot.py
