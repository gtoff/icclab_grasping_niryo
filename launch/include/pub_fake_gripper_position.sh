#!/bin/bash
rostopic pub -s -r 10 /joint_states sensor_msgs/JointState "header: 'auto'
name: [gripper_left_finger_joint, gripper_right_finger_joint]
position: [0,0]
velocity: [0,0]
effort: [0,0]"
