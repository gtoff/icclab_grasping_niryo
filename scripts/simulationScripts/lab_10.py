#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def niryo_demo():
        # Start moveit commander and initialize ROS node
	moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_niryo_robot')

        # Configure move_group
	group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_planning_time(10)

	#
	# Your code
	#

        # Stop moveit
	moveit_commander.roscpp_shutdown()



if __name__ == '__main__':
        try:
                niryo_demo()
        except rospy.ROSInterruptException:
		pass
