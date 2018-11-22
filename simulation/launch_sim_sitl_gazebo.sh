#!/bin/bash
##########################################
# Changelog
# Author: Frederik Mazur Andersen (fande14@student.sdu.dk)
# first version - FMA
##########################################

# Drone start position, is currently middle of scene on ground
export PX4_HOME_LAT=55.4720422
export PX4_HOME_LON=10.4147126

# source ros/gazebo stuff
source /opt/ros/melodic/setup.bash

# Go to firmware folder ### CHANGE PATHS FOR YOUR SETUP
cd Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# launch virtual port in background, ttyV6 is the lora port
socat -d -d pty,raw,echo=0,link=/tmp/ttyV5 pty,raw,echo=0,link=/tmp/ttyV6 &

# Bridge udp and serial
socat -d udp4-listen:14540 open:/tmp/ttyV5,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b115200,crnl &

# Launch SITL with gazebo, without mavros
roslaunch px4 posix_sitl_irlock.launch

