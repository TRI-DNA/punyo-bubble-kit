#!/bin/bash
source set_env.sh
PRESSURE_TOPIC=`udevadm info --name=/dev/ttyACM0 | grep SERIAL_SHORT | awk -F= '{  printf "/bubble_%s/pressure",$NF }'`
ros2 launch punyo bubble_music_launch.py pressure_parameter:=$PRESSURE_TOPIC
