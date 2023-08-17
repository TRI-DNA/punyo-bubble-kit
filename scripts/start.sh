#!/bin/bash
# This script is for convenience to invoke the pressure, camera, flow and rqt scripts.
# Be sure to update your WORKING_DIR

WORKING_DIR="/home/dna/punyo-bubble-kit/scripts"

invocation=`ps -o stat= -p $PPID`

if [ $invocation == "Ss" ]; then
    echo "To make the tabs easier to close (so you can just close a single window), launch this script with:"
    echo "$ gnome-terminal --window -- ./start.sh"
fi

header="================================================================================"
mcu_msg1="When configuring the ROS reference file, use \$ python3 prepare_ref.py "
mcu_msg2="You may need to close the window and then re-run the launcher to make sure that the newly written ref file is loaded."
pressure_msg="If you are only seeing an INFO set_log_level message, press the reset button on the microcontroller."
pressure_topic="Once the pressure topic is published, it will be in the format of /bubble_serial_number/pressure/data"
camera_msg="Warning messages related to depth_module.power_line_frequency are okay."
flow_msg="In one of the FARNEBECK/RAFT flow visualization windows, type \'c\' to clear the reference image, \'s\' to save the current image to the reference set of images"
flow_topics="/bubble_1/force/data[0] and /bubble_1/force/data[1]"
rqt_msg="To reload the default Punyo visualization, use Perspectives \> Import \> punyo.perspective. You may need to use a unique name."
SERIAL_ID=`udevadm info --name=/dev/ttyACM0 | grep SERIAL_SHORT | awk -F= '{ print $NF }'`
plot_msg="To add the plot of the pressure data, make sure that the MCU LED is green, the plot is refreshed to get the current topics list, and then enter /bubble_$SERIAL_ID/pressure/data. If you just start typing in the topic field, it should autofill matches. You may scroll to the bottom of the list to find where the topic name gets sorted."
led_msg="Set the LED brightness"
press="Check the pressure data, then press any key..."
gnome-terminal --tab --title "1. Configure" -- bash -c "cd $WORKING_DIR; echo $mcu_msg1; echo $mcu_msg2; echo $pressure_topic; exec bash;"
gnome-terminal --tab --title "2. Pressure" -- bash -c "echo $header; echo $pressure_msg; echo $header; cd $WORKING_DIR; ./start_pressure1.sh;"
gnome-terminal --tab --title "3. Camera  " -- bash -c "echo $header; echo $camera_msg; echo $header; cd $WORKING_DIR; ./start_camera1.sh"
gnome-terminal --tab --title "4. Flow    " -- bash -c "echo $header; echo $flow_msg; echo $flow_topics; echo $header; cd $WORKING_DIR; ./start_flow.sh"
gnome-terminal --tab --title "5. RQT     " -- bash -c "echo $header; echo $rqt_msg; echo $plot_msg; echo $header; cd $WORKING_DIR; echo $press; read -p "_" tmp; ./start_rqt.sh"
gnome-terminal --tab --title "6. LED     " -- bash -c "echo $header; echo $led_msg; echo $header; cd $WORKING_DIR; ./start_led.sh"
