# Pressure System Board

## Arduino IDE Installation
### Overview
To upload new code to the microcontroller, you can use the Arduino IDE with the following libraries:	

	* Adafruit MPRLS library
	* Adafruit Neopixel library
	* Micro-ros library (custom build)

### Software Installation

* Install Arduino 1.8.19
* Install the following libraries:
	* micro_ros_arduino
		* Micro ROS Arduino Project page: https://github.com/micro-ROS/micro_ros_arduino
		* You will need to build the precompiled library. [See the configuration parameters required](#How-to-build-the-precompiled-library) or use the one provided [here](https://github.shared-services.aws.tri.global/robotics/punyo-bubble/tree/main/pressure_sensors/libraries).
		* To install, Navigate to Sketch > Include Library > Add .ZIP library
		* Select the zip file.
	* Adafruit MPRLS
		* to install, open the Library Manager under Sketch > Include Library > Manage Libraries...
		* Search for `Adafruit MPRLS` to install it
		* Choose Install All to get the dependencies.
	* Adafruit Neopixel
		* to install, open the Library Manager under Sketch > Include Library > Manage Libraries...
		* Search for `Adafruit Neopixel` to install it
	* Ardunino Unique ID
		* to install, open the Library Manager under Sketch > Include Library > Manage Libraries...
		* Search for `ArduinoUniqueID` to install it
* Add support for the Adafruit Qt Py microcontoller board
	* Navigate to File > Preferences > Additional Board Manager URLs
		* Add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json to the list of board URLs.
	* Navigate to Tools > Board (...) > Boards Manager...
		* Search for `Adafruit SAMD`
	* For additional details, refer to https://learn.adafruit.com/adafruit-qt-py/arduino-ide-setup for more help

## Reprogramming the microcontroller
If you have installed the Arduino IDE (listed as optional above in the installation section), you should have all of the tools to recompile the code and upload it to the MC.

1. Open the file `punyo-bubble/pressure_sensors/micro_ros_bubble/micro_ros_bubble.ino`
2. Under `Tools > Board`, Choose `Adafruit SAMD (...) Boards` > `Adafruit Qt Py M0 (SAMD21)`
3. Under `Tools > Port`, Choose the port that your microcontroller is connected to, typically something like `/dev/ttyACM0` 
4. If all of the dependencies have been satisified, you can choose `verify/compile`(just to compile without uploading) or `upload` (to compile and then upload).
5. While the new code is being upload, you may see a popup showing that the board has reconnected as a device 'QTPY_BOOT'
6. If successful (you may see a warning message, ok to proceed), then MC will start to run the updated code and you can refer to patterns listed below to determine the state.
7. As this point you can run the MicroXRCEAgent (as mentioned in the `Getting Pressure Data into ROS 2` section), to get the data in ROS.

## Troubleshooting using the on-board neopixel (RGB LED)
You can check the state of the micro-ros client by checking the on-board neopixel (RGB LED). 

| Color | Status|
|-----------| ----------------------------------|
| Solid Orange | Initializing (up to 5 seconds)|
|Solid Green|		Connected.|
|Solid White|Connected. LED Brightness adjusted and is reflected in the brightness of the on-board neopixel LED.|
|Solid Blue|No connection to agent or invalid refs file (check console log for errors). Run agent, press reset if necessary.|
|Flashing Red|Lost connection to agent. Restart agent, press reset if necessary.|

## How to build the precompiled library
Option 1:
Use the micro_ros_setup project to create the build system.

Following the instructions here to build the setup package. https://github.com/micro-ROS/micro_ros_setup#building

After completing the those steps (ending with `source install/local_setup.bash`)

Do the following to build the firmware:
```
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

# Set the toolchain prefix (gcc, etc... will be appended to this); Check your Arduino compilation logs if you need to confirm the path
export TOOLCHAIN_PREFIX=<path_to_arduino>/.arduino15/packages/adafruit/tools/arm-none-eabi-gcc/9-2019q4/bin/arm-none-eabi-

# NOTE: the cmake and meta files used to build the firmware are in this repo. Check the path.
ros2 run micro_ros_setup build_firmware.sh ../punyo-bubble/library_generation/cortex_m0_toolchain.cmake ../punyo-bubble/library_generation/colcon_verylowmem.meta
```

Option 2: 
Use the micro_ros_arduino project to build the library using docker 

Docker installation
https://docs.docker.com/engine/install/ubuntu/

```
export ROS_DISTRO=galactic
git clone -b $ROS_DISTRO git@github.com:micro-ROS/micro_ros_arduino.git
cd micro_ros_arduino

docker pull microros/micro_ros_static_library_builder:$ROS_DISTRO

# This probably won't work because the default toolchain for the cortex_m0 does not support the Arduino board 
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:$ROS_DISTRO -p cortex_m0

# Alternative: download the build tools and then run the builder manually.
docker run -it -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras --entrypoint /bin/bash microros/micro_ros_static_library_builder:$ROS_DISTRO

cd uros
wget https://github.com/adafruit/arduino-board-index/releases/download/build-tools/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
tar -xvf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2

apt update
source /opt/ros/galactic/setup.bash 
source install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
```

On your host, edit the `library_generation.sh` script located in the `micro_ros_arduino/extras/library_generation` folder.

```
# Set the prefix to match the unzipped tools (done a few steps above) 
export TOOLCHAIN_PREFIX=/uros_ws/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-
```

Continuing in the container...
```
ros2 run micro_ros_setup build_firmware.sh /project/extras/library_generation/cortex_m0_toolchain.cmake /project/extras/library_generation/colcon_verylowmem.meta

# The `libmicroros.a` is the updated pre-compiled library. You may copy it back to your docker host (`project/` is mapped to what was passed to the container launch command)
# (e.g., if you just want the new library)
cp firmware/build/libmicroros.a /project/
```

Or follow the steps as shown in the `library_generation.sh` script to get the source files, etc... 

Either way, you will still need to re-zip the files back into a .zip to use the 
Include Library > Add .ZIP library option in the Arduino IDE.

---
Punyo Soft-Bubble Sensor - Copyright 2022 Toyota Research Institute. All rights reserved.