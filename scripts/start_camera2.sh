#!/bin/bash
set -e

SERIAL_NUMBER=${SERIAL_NUMBER:-"127122270513"}              # Connect to the camera with serial number
EXIT_IF_CAMERA_NOT_FOUND=${EXIT_IF_CAMERA_NOT_FOUND:-false} # true: exit if not found; false: keep running if not found
CAMERA_NAME=${CAMERA_NAME:-"bubble_1"}                      # Set the ROS2 topic name

source /opt/ros/galactic/setup.bash
# Set param AFTER sourcing ROS setup, otherwise doesn't seem to always work
ROS_LOCALHOST_ONLY=1                                        # 1: keep published data local, 0: broadcast
source ../ros2_ws/install/setup.bash

die (){
  echo "$@" 1>&2
  exit 1
}

if rs-enumerate-devices -s | grep $SERIAL_NUMBER;  then
  echo "Found camera"
else
  echo -e "\e[31mCamera with serial number $SERIAL_NUMBER not found\e[0m"
  if [ "$EXIT_IF_CAMERA_NOT_FOUND" = true ]; then
    die "Check your camera serial number or set EXIT_IF_CAMERA_NOT_FOUND to false"
  else
    echo "Connecting to any camera..."
  fi
fi

CAMERA_PARAMS="camera_name:=$CAMERA_NAME serial_number:=$SERIAL_NUMBER"
GENERAL_PARAMS="log_level:=WARN pointcloud.enable:=false"
COLORIZER_PARAMS="colorizer.enable:=true colorizer.color_scheme:=3"
DEPTH_PARAMS="depth_module.enable_auto_exposure.1:=false depth_module.profile:=640x480x30 depth_module.exposure:=1000 depth_module.gain:=128"
echo "Using parameters: " $CAMERA_PARAMS $GENERAL_PARAMS $COLORIZER_PARAMS $DEPTH_PARAMS
# Some warnings are logged by rs_launch.py about parameters not in the correct range (unrelated to the ones in this script)
ros2 launch realsense2_camera rs_launch.py $CAMERA_PARAMS $GENERAL_PARAMS $COLORIZER_PARAMS $DEPTH_PARAMS
