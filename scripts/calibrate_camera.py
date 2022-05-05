#!/usr/bin/env python3

from niryo_one_python_api.niryo_one_api import *
from pprint import pprint
import sys
import yaml
import subprocess
import copy
import rospy
import time
import tf
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
import os

def calibrate_camera():
	rospy.init_node('camera_calibration')
	ar_marker_id = "13"
	ar_pose = [0.003, -0.23, 0.0, 0.707, 0.0, 0.0, 0.707]

	listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
	#transformer = tf.TransformerROS(True, rospy.Duration(10.0))

	time.sleep(5)
	t = rospy.Time.now()

	listener.waitForTransform("ground_link", "camera_color_optical_frame", t, rospy.Duration(4.0))
	(trans, quat) = listener.lookupTransform("ground_link", "camera_color_optical_frame", t)

	# pose_msg = geometry_msgs.msg.PoseStamped()
	# pose_msg.header.frame_id = "ar_marker_" + ar_marker_id
	# pose_msg.header.stamp = t
	# pose_msg.pose.position.x = trans[0]
	# pose_msg.pose.position.y = trans[1]
	# pose_msg.pose.position.z = trans[2]
	# pose_msg.pose.orientation.x = quat[0]
	# pose_msg.pose.orientation.y = quat[1]
	# pose_msg.pose.orientation.z = quat[2]
	# pose_msg.pose.orientation.w = quat[3]
	# listener.waitForTransform("ar_marker_"+ar_marker_id, "ground_link", t, rospy.Duration(4.0))
	# pose_final = listener.transformPose("ground_link", pose_msg)

	#trans = (pose_final.pose.position.x, pose_final.pose.position.y, pose_final.pose.position.z)
	#quat  = (pose_final.pose.orientation.x, pose_final.pose.orientation.y, pose_final.pose.orientation.z, pose_final.pose.orientation.w)

	print("Ground_link-to-camera_link_frame transform: [x y z | x y z w]")
	pprint(trans+quat)

	buf = []
	buf.append("<launch>\n")
	buf.append('<node pkg="tf" type="static_transform_publisher" name="camera_color_optical_frame_broadcaster" args="')
	for x in trans:
		buf.append(str(x) + " ")
	for x in quat:
		buf.append(str(x) + " ")
	buf.append('ground_link camera_color_optical_frame 100" />\n')
	buf.append("</launch>\n")

	with open(os.path.dirname(__file__) + '/../launch/static_camera_transformation_publisher.launch', 'w') as outfile:
		outfile.write(''.join(buf))
		outfile.close()

	print("Calibration completed. You can stop this script and launch :\n roslaunch icclab_grasping_niryo static_camera_transformation_publisher.launch")

if __name__ == '__main__':
	calibrate_camera()
