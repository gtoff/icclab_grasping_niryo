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


def test_gripper1():
	#Approach vector and offset distance to compensate for gripper length

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_niryo_robot')

	n = NiryoOne()
	#n.calibrate_auto()
	#n.activate_learning_mode(False)
	n.change_tool(TOOL_GRIPPER_1_ID)
   	n.open_gripper(TOOL_GRIPPER_1_ID, 200)
	print("[GRIPPER 1 OPENED]")
	n.close_gripper(TOOL_GRIPPER_1_ID, 200)
	print("[GRIPPER 1 CLOSED]")
	moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
	try:
		test_gripper1()
	except rospy.ROSInterruptException:
		n.activate_learning_mode(True)
