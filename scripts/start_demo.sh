#!/bin/bash
cd ../ros2_ws
source /opt/ros/galactic/setup.bash
export ROS_LOCALHOST_ONLY=1
source install/setup.bash
ros2 launch punyo bubble_music_launch.py pressure_parameter:=/bubble_35A4E5A250555733362E3120FF091A1E/pressure
