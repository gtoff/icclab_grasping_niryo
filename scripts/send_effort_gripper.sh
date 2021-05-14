#!/bin/sh
#
# Send a value to change the opening of an effort controlled gripper
#

if [ "$#" -ne 2 ]
  then echo "usage: send_effort_gripper.sh <left_grippoer_effort_float_value> <right_gripper_effort_float_value> "
else
  rostopic pub -1 /gripper_left_controller/command std_msgs/Float64 -- $1 &
  rostopic pub -1 /gripper_right_controller/command std_msgs/Float64 -- $2
fi
