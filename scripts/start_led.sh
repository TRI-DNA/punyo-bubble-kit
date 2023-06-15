#!/bin/bash
source set_env.sh

while [ 1 ]
do
    read -p 'LED Brightnesss 0-255: ' BRIGHTNESS
    LED_TOPIC=`udevadm info --name=/dev/ttyACM0 | grep SERIAL_SHORT | awk -F= '{  printf "/bubble_%s/led",$NF }'`
    echo "Publishing to topic: $LED_TOPIC"
    ros2 topic pub -1 $LED_TOPIC std_msgs/Int32 "data: $BRIGHTNESS"
done