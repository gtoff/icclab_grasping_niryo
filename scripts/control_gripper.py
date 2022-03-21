#!/usr/bin/env python3

# from niryo_one_python_api.niryo_one_api import *
import sys
import rospy
import time
import argparse
from std_msgs.msg import Empty, Float64
from niryo_one_msgs.msg import ToolActionGoal

# GRIPPER_ID=TOOL_GRIPPER_1_ID

class control_gripper:

  def __init__(self, sim):
    #Approach vector and offset distance to compensate for gripper length
    rospy.init_node('control_niryo_gripper')
    if (sim):
      self.gripper_pub1 = rospy.Publisher('/gripper_left_controller/command', Float64, queue_size=1)
      self.gripper_pub2 = rospy.Publisher('/gripper_right_controller/command', Float64, queue_size=1)
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

  def open(self, data, sim):
      rospy.loginfo("Received gripper open request")
      if (sim):
        self.send_to_sim_gripper(-0.5)
      else:
        # self.n.open_gripper(GRIPPER_ID, 300)
        msg = self.getToolActionGoal()
        self.action_pub.publish(msg)
      print("[GRIPPER OPENED]")

  def close(self, data, sim):
      rospy.loginfo("Received gripper close request")
      if (sim):
        self.send_to_sim_gripper(0.5)
      else:
        # self.n.close_gripper(GRIPPER_ID, 300)
        msg = self.getToolActionGoal()
        msg.goal.cmd.cmd_type = 2 # close
        self.action_pub.publish(msg)
      print("[GRIPPER CLOSED]")

  
  
  def send_to_sim_gripper(self, value):
      rospy.sleep(0.2)
      self.gripper_pub1.publish(-value)
      self.gripper_pub2.publish(value)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Control a Niryo gripper in sim / hw')
  parser.add_argument('--sim', action="store_true",
                      help='Simulated robot: True')

  args = parser.parse_args() 
  print("Starting gripper controller SIMULATED = ", args.sim) 

  try:
    control_gripper(args.sim)
  except rospy.ROSInterruptException:
    n.activate_learning_mode(True)
