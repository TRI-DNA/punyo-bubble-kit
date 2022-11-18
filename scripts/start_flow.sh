#!/bin/bash
cd ../ros2_ws
source /opt/ros/galactic/setup.bash
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
ros2 run punyo bubble_image_subscriber --ros-args --params-file ../ros2_ws/src/punyo/punyo/config/bubble.yaml