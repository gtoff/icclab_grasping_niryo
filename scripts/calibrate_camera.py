#!/usr/bin/env python3

from niryo_one_python_api.niryo_one_api import *
from pprint import pprint
import sys
import yaml
import subprocess
import copy
import rospy
import time
import tf2_ros
import tf2_geometry_msgs
import math
from math import pi
from std_msgs.msg import String
import os

SOURCE_FRAME = "ground_link"
TARGET_FRAME = "camera_link"

def calibrate_camera():
	rospy.init_node('camera_calibration')
	tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
	tf_listener = tf2_ros.TransformListener(tf_buffer)
	t = tf_buffer.lookup_transform(
		SOURCE_FRAME, 
		TARGET_FRAME, 
		rospy.Time(0), #get the tf at first available time
		rospy.Duration(5.0)) #wait for 1 second

	print(t)
	
	buf = []
	buf.append("<launch>\n")
	buf.append('<node pkg="tf2_ros" type="static_transform_publisher" name="camera_pose_broadcaster" args="')
	buf.append(str(t.transform.translation.x) + " " + str(t.transform.translation.y) + " " + str(t.transform.translation.z) + " ")
	buf.append(str(t.transform.rotation.x) + " " + str(t.transform.rotation.y) + " " + str(t.transform.rotation.z) + " " + str(t.transform.rotation.w) + " ")
	buf.append(t.header.frame_id)
	buf.append(" ")
	buf.append(t.child_frame_id)
	buf.append('" />\n')
	buf.append("</launch>\n")

	print(os.path.dirname(__file__))
	with open(os.path.abspath(os.path.dirname(__file__)) + os.path.sep + '../launch/static_camera_transformation_publisher.launch', 'w') as outfile:
		outfile.write(''.join(buf))
		outfile.close()

	print("Calibration completed. You can stop this script and launch :\n roslaunch icclab_grasping_niryo static_camera_transformation_publisher.launch")

if __name__ == '__main__':
	calibrate_camera()
