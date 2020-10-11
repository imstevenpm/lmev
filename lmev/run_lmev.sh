#!/bin/bash 

# Cleans workspace
killall rosmaster
killall gzserver
killall gzclient

# Source the setup.bash
. ~/lmev/devel/setup.bash

# Run the necessary commands for the correct initialization
# You can add your own commands here
# --tab -e "yourcommand"
gnome-terminal --tab -e "roslaunch lmev_gazebo lmev.launch" --tab -e "rosrun image_view image_view image:=/lent/image_raw" --tab -e "rosrun lmev_gazebo Main_Controller.py"
