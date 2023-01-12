source /opt/ros/galactic/setup.bash
# Set param AFTER sourcing ROS setup, otherwise doesn't seem to always work
export ROS_LOCALHOST_ONLY=1           # 1: keep published data local, 0: broadcast
source ../ros2_ws/install/setup.bash