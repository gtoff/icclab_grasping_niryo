#!/bin/bash

# dependencies: timed roslaunch
# https://github.com/MoriKen254/timed_roslaunch

rosrun timed_roslaunch timed_roslaunch.sh 0 icclab_grasping_niryo base_world_model.launch &

rosrun timed_roslaunch timed_roslaunch.sh 10 icclab_grasping_niryo controllers.launch &

rosrun timed_roslaunch timed_roslaunch.sh 20 niryo_one_bringup robot_commander.launch &

rosrun timed_roslaunch timed_roslaunch.sh 30 icclab_grasping_niryo rviz_grasps_filter.launch &

sleep 40

read -n 1 -s -r -p "Press any key to continue"

echo -n "killing gserver..."
pkill -9 gzserver
sleep 5
echo -n "killing gzclient..."
pkill -9 gzclient
sleep 5
echo -n "killing all nodels..."
## THIS ONE LATER MIGHT BREAK THE CODE
## SHOULD KILL THE NODES 1 by 1 EXPLICTLY BY NAME 
rosnode kill -a
sleep 5
echo -n "killing rosmaster..."
pkill -9 rosmaster
echo -n "PRESS Ctrl-C TO TERMINATE THIS PROCESS"
echo -n "Yatta!!!!!"


