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
	ar_pose = [0.03, -0.24, 0.0, 1.57, 0.0, 0.0]

	aruco_static_transform = subprocess.Popen(
				["rosrun", "tf", "static_transform_publisher", str(ar_pose[0]), str(ar_pose[1]), str(ar_pose[2]), 
				str(ar_pose[3]), str(ar_pose[4]), str(ar_pose[5]), "ar_marker_"+ar_marker_id, "ground_link", "100"])

	listener = tf.TransformListener()
	time.sleep(3)
	frame_list = listener.getFrameStrings()

	listener.waitForTransform("camera_color_optical_frame", "ground_link", rospy.Time(), rospy.Duration(2.0))
	(trans, quat) = listener.lookupTransform("camera_color_optical_frame", "ground_link", rospy.Time(0))

	print("Camera-to-ground transform: [x y z | x y z w]")
	pprint(trans+quat)

	aruco_static_transform.kill()

	with open('camera_params.yaml', 'w') as outfile:
		yaml.dump(trans+quat, outfile, explicit_start=True, default_flow_style=False)

if __name__ == '__main__':
	calibrate_camera()
