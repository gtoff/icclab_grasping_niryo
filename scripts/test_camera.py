#!/usr/bin/env python3

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

def test_camera():
	rospy.init_node("camera_test")

	bcaster = tf2_ros.StaticTransformBroadcaster()

	with open('camera_params.yaml', 'r') as infile:
		cam_pose = yaml.load(infile)

	transform_msg = geometry_msgs.msg.TransformStamped()
	transform_msg.header.frame_id = "ground_link"
	transform_msg.child_frame_id  = "camera_link"
	transform_msg.transform.translation.x = cam_pose[0]
	transform_msg.transform.translation.y = cam_pose[1]
	transform_msg.transform.translation.z = cam_pose[2]
	transform_msg.transform.rotation.x = cam_pose[3]
	transform_msg.transform.rotation.y = cam_pose[4]
	transform_msg.transform.rotation.z = cam_pose[5]
	transform_msg.transform.rotation.w = cam_pose[6]
	bcaster.sendTransform(transform_msg)
	rospy.spin()
	#time.sleep(5)


if __name__ == "__main__":
	test_camera()
