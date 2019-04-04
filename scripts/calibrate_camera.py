#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
from pprint import pprint
import sys
import yaml
import subprocess
import copy
import rospy
import time
import tf
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def calibrate_camera():
	rospy.init_node('camera_calibration')
	ar_marker_id = "13"
	ar_pose = [0.003, -0.23, 0.0, 0.707, 0.0, 0.0, 0.707]

	bcaster = tf2_ros.StaticTransformBroadcaster()
	transform_msg = geometry_msgs.msg.TransformStamped()
	transform_msg.header.frame_id = "ar_marker_" + ar_marker_id
	transform_msg.child_frame_id  = "ground_link"
	transform_msg.transform.translation.x = ar_pose[0]
	transform_msg.transform.translation.y = ar_pose[1]
	transform_msg.transform.translation.z = ar_pose[2]
	transform_msg.transform.rotation.x = ar_pose[3]
	transform_msg.transform.rotation.y = ar_pose[4]
	transform_msg.transform.rotation.z = ar_pose[5]
	transform_msg.transform.rotation.w = ar_pose[6]
	bcaster.sendTransform(transform_msg)

	listener = tf.TransformListener()

	listener.waitForTransform("camera_color_optical_frame", "ground_link", rospy.Time(), rospy.Duration(2.0))
	(trans, quat) = listener.lookupTransform("ground_link", "camera_color_optical_frame", rospy.Time(0))

	print("Camera-to-ground transform: [x y z | x y z w]")
	pprint(trans+quat)

	with open('camera_params.yaml', 'w') as outfile:
		yaml.dump(trans+quat, outfile, explicit_start=True, default_flow_style=False)

if __name__ == '__main__':
	calibrate_camera()
