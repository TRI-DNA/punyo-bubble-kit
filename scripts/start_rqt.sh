#!/bin/bash
cd ../ros2_ws
source /opt/ros/galactic/setup.bash
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
rqt
