#!/usr/bin/env python3

# from niryo_one_python_api.niryo_one_api import *
import sys
import rospy
import time
import actionlib
import argparse
from std_msgs.msg import Empty, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from niryo_one_msgs.msg import ToolActionGoal

# GRIPPER_ID=TOOL_GRIPPER_1_ID

class control_gripper:

  def __init__(self, name, sim):
    #Approach vector and offset distance to compensate for gripper length
    rospy.init_node(name)
    if (sim):
      self.gripper_pub1 = rospy.Publisher('/gripper_left_controller/command', Float64, queue_size=1)
      self.gripper_pub2 = rospy.Publisher('/gripper_right_controller/command', Float64, queue_size=1)
      # self.gripper_client = rospy.Publisher('/gripper_follow_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryAction, queue_size=1)
      self.gripper_client = actionlib.SimpleActionClient('/gripper_follow_joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    else:
      # self.n = NiryoOne()  
      # rospy.sleep(0.5)
      # self.n.change_tool(GRIPPER_ID)
      self.action_pub = rospy.Publisher("/niryo_one/tool_action/goal", ToolActionGoal, queue_size=1)
    subOpen = rospy.Subscriber("/gripper/open", Empty, callback=self.open, queue_size=1, callback_args=sim)
    subClose = rospy.Subscriber("/gripper/close", Empty, callback=self.close, queue_size=1, callback_args=sim)
    rospy.spin()


  def getToolActionGoal(self):
      msg = ToolActionGoal()
      msg.goal.cmd.tool_id = 11
      msg.goal.cmd.cmd_type = 1 # open
      msg.goal.cmd.gripper_close_speed = 300
      msg.goal.cmd.gripper_open_speed = 300
      msg.goal.cmd.activate = False
      msg.goal.cmd.gpio = 0
      return msg


  # https://github.com/myyerrol/xm_arm_workspace/blob/master/xm_arm_trajectory_control/scripts/xm_arm_trajectory_move_test.py
  def getFollowJointTrajectoryActionGoal(self, goal):
      gripper_joint_names = ['gripper_left_finger_joint',
                             'gripper_right_finger_joint']

      gripper_trajectory = JointTrajectory()
      gripper_trajectory.joint_names = gripper_joint_names
      gripper_trajectory.points.append(JointTrajectoryPoint())
      gripper_trajectory.points[0].positions = goal;
      gripper_trajectory.points[0].time_from_start = rospy.Duration(1)
      rospy.loginfo("Preparing for moving the gripper to goal position!")
      rospy.sleep(1)
      gripper_goal_pos = FollowJointTrajectoryGoal()
      gripper_goal_pos.trajectory = gripper_trajectory
      gripper_goal_pos.goal_time_tolerance = rospy.Duration(0)

      return gripper_goal_pos;


  def open(self, data, sim):
      rospy.loginfo("Received gripper open request")
      print("Received gripper open request")
      if (sim):
        self.send_to_sim_gripper([0, 0])
      else:
        # self.n.open_gripper(GRIPPER_ID, 300)
        msg = self.getToolActionGoal()
        self.action_pub.publish(msg)
      print("[GRIPPER OPENED]")


  def close(self, data, sim):
      rospy.loginfo("Received gripper close request")
      print("Received gripper open request")
      if (sim):
        self.send_to_sim_gripper([-0.015, 0.015])
      else:
        # self.n.close_gripper(GRIPPER_ID, 300)
        msg = self.getToolActionGoal()
        msg.goal.cmd.cmd_type = 2 # close
        self.action_pub.publish(msg)
      print("[GRIPPER CLOSED]")
  
  
  def send_to_sim_gripper(self, goal):
      rospy.sleep(0.2)
      gripper_goal_pos = self.getFollowJointTrajectoryActionGoal(goal)
      self.gripper_client.send_goal(gripper_goal_pos)
      rospy.loginfo("Send goal to the trajectory server successfully!")
      self.gripper_client.wait_for_result()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Control a Niryo gripper in sim / hw')
  parser.add_argument('--sim', action="store_true",
                      help='Simulated robot: True')                  

  #args = parser.parse_args() 
  args, unknown = parser.parse_known_args()
  print("Starting gripper controller SIMULATED = ", args.sim) 

  try:
    control_gripper('control_niryo_gripper', args.sim)
  except rospy.ROSInterruptException:
    n.activate_learning_mode(True)
