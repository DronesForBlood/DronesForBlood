#!/bin/bash
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@${mavroshost:-localhost}:14557" &
sleep 2
roslaunch gcs_master gcs.launch
