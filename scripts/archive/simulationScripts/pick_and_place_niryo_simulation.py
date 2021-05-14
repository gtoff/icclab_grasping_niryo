#!/usr/bin/env python3
import sys
sys.path.append('/home/ros/catkin_ws/src/icclab_grasping_niryo/scripts/')
import rospy
import numpy as np
import copy
import tf
import gc
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd_ros.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header, ColorRGBA
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
from tf.transformations import *
import geometry_msgs.msg 
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, degrees
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
import time
from send_gripper import gripper_client_2
from tf import TransformListener
import copy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import sensor_msgs
import actionlib
from filter_pointcloud_client import call_pointcloud_filter_service
from moveit_commander import MoveGroupCommander, RobotCommander
from copy import deepcopy
from pointcloud_operations import create_mesh_and_save
from sensor_msgs import point_cloud2
from show_pose_marker import place_marker_at_pose
from std_srvs.srv import Empty

class GpdPickPlace(object):
    grasps = []
    mark_pose = True
    grasp_offset = -0.04
    grasps_received = False

    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_callback)
        # We do not have marker in this case 
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
        self.p = PickPlaceInterface(group="arm", ee_group="gripper", verbose=True, ns="")
        self.tf = tf.TransformListener()
   
    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasps_received = True
        if (len(msg.grasps)==0):
            pevent("No grasps found, aborting!")
        else:
            pevent("Received new grasps")

    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
        while (self.grasps_received == False):
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_msgs(self, selected_grasps):
            self.grasps = []
            formatted_grasps = []
            cont = 0
            filtered_orientation = 0
            for i in range(0, len(selected_grasps)):
                z_axis_unit = (0, 0, 1)
                ap_axis = (selected_grasps[i].approach.x, selected_grasps[i].approach.y, selected_grasps[i].approach.z)
                angle = numpy.dot(z_axis_unit, ap_axis)
                if (angle >= 0):
                    # filter it out, because grasp coming from below the ground
                    filtered_orientation += 1
                    print(repr(filtered_orientation) + " Grasp filtered because coming from underneath the ground")
                    continue
                
                tf_listener_.waitForTransform('/camera_optical_frame', '/base_link', rospy.Time(), rospy.Duration(2.0))
                quat = self.trans_matrix_to_quaternion(selected_grasps[i])
                gp = PoseStamped()
                gp.header.frame_id = "camera_optical_frame"
                gp.pose.position.x = selected_grasps[i].position.x + self.grasp_offset * selected_grasps[i].approach.x
                gp.pose.position.y = selected_grasps[i].position.y + self.grasp_offset * selected_grasps[i].approach.y
                gp.pose.position.z = selected_grasps[i].position.z + self.grasp_offset * selected_grasps[i].approach.z
                gp.pose.orientation.x = float(quat.elements[1])
                gp.pose.orientation.y = float(quat.elements[2])
                gp.pose.orientation.z = float(quat.elements[3])
                gp.pose.orientation.w = float(quat.elements[0])

                translated_pose = tf_listener_.transformPose("ground_link", gp)
                g = Grasp ()
                g.id = "grasp_id"
                g.grasp_pose = translated_pose 
                g.allowed_touch_objects = ["<octomap>","obj"]
                formatted_grasps.append(g)

            # Sort grasps using z (get higher grasps first)
            formatted_grasps.sort(key=lambda grasp: grasp.grasp_pose.pose.position.z, reverse=True)
            return formatted_grasps

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                      [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                      [grasp.approach.z, grasp.binormal.z, grasp.axis.z]])
        return Quaternion(matrix=r)

    def pick_two_steps(self, grasps_list, verbose=False):
        pevent("Two step pick sequence started")
        
	# Add object mesh to planning scene
        self.add_object_mesh()
        group.set_goal_tolerance(0.01)
        group.set_planning_time(5)
        cont_c = 0
        for single_grasp in grasps_list:
            if self.mark_pose:
                place_marker_at_pose(self.marker_publisher, single_grasp.grasp_pose)
                rospy.sleep(1)
            pevent("Planning grasp:")
            single_grasp.grasp_pose.pose.position.z = single_grasp.grasp_pose.pose.position.z + 0.005 
            pprint(single_grasp.grasp_pose)
            group.set_start_state_to_current_state()
            group.set_pose_target(single_grasp.grasp_pose.pose)
            #plan = group.plan()
            plan_success, plan, planning_time, error_code = group.plan()

            if (len(plan.joint_trajectory.points) != 0):
                inp = input("Have a look at the planned motion. Do you want to proceed? y/n: ")
                if (inp == 'y'):
                    pevent("Executing grasp: ")
                    pick_result = group.execute(plan, wait=True)
                    if pick_result == True:
                        group.stop()
                        group.clear_pose_targets()
                        group.set_start_state_to_current_state()
                        if self.mark_pose:
                            place_marker_at_pose(self.marker_publisher, grasps_list[cont_c].grasp_pose)
                            rospy.sleep(1)
                        group.set_pose_target(grasps_list[cont_c].grasp_pose.pose)
                        plan2 = group.go()
                        return single_grasp
                    else:
                        group.stop()
                        group.clear_pose_targets()
                elif (inp == 'exit'):
                    group.stop()
                    group.clear_pose_targets()
                    exit(1)
            cont_c += 1
        self.grasps = []

    def place(self, place_pose):
        pevent("Place sequence started")
        place_result = self.p.place_with_retry("obj", place_pose, support_name="<octomap>", planning_time=9001, goal_is_eef=True)

    def add_object_mesh(self):
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = "camera_optical_frame"
        obj_pose.pose.position.x = 0
        obj_pose.pose.position.y = 0
        obj_pose.pose.position.z = 0
        obj_pose.pose.orientation.x = 0
        obj_pose.pose.orientation.y = 0
        obj_pose.pose.orientation.z = 0
        obj_pose.pose.orientation.w = 1
        translated_pose = tf_listener_.transformPose("ground_link", obj_pose)
	
	#remove collision object from previous run
        planning.removeCollisionObject("obj")
        rospy.sleep(1)
        planning.addMesh("obj", translated_pose.pose, "object.stl")
        print("Collision object is:")
        rospy.sleep(3)
        pprint(planning.getKnownCollisionObjects())

    def move_to(self, x, y, z, qx, qy, qz, qw, drop):
        pevent("Dropping object on robot")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        group.set_start_state_to_current_state()
        group.set_goal_tolerance(0.05)
        group.set_pose_target(pose_goal)

        #plan = group.plan()
        plan_success, plan, planning_time, error_code = group.plan()
        rospy.sleep(1)
        cont_plan_drop = 0
        while ((len(plan.joint_trajectory.points) == 0) and (cont_plan_drop < 10)):
            #plan = group.plan()
            plan_success, plan, planning_time, error_code = group.plan()
            rospy.sleep(1)
            cont_plan_drop += 1
        if (len(plan.joint_trajectory.points) != 0):
            pevent("Executing dropping: ")
            result = group.execute(plan, wait=True)
            rospy.sleep(1)
            if drop == True:
                pevent("Dropping successful!")
                result = gripper_client_2(-0.5)
                rospy.sleep(3)
                print("Gripper opened")
                group.detach_object("obj")
                group.stop()
                group.clear_pose_targets()

    def wait_for_mesh_and_save(self):
      pinfo("Subscribing to pointcloud to generate mesh")
      self.obj_pc_subscriber = rospy.Subscriber("/cloud_indexed_pc_only", sensor_msgs.msg.PointCloud2 , self.obj_pointcloud_callback)

    def obj_pointcloud_callback(self, msg):
      pinfo("Pointcloud received")
      cloud = []
      for p in point_cloud2.read_points(msg, skip_nans=True):
                cloud.append([p[0], p[1], p[2]])
      create_mesh_and_save(cloud)
      pinfo("Mesh generated")
      self.obj_pc_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("gpd_pick_and_place",anonymous=True)
    tf_listener_ = TransformListener()
    pnp = GpdPickPlace(mark_pose=True)
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name, robot_description="/robot_description", ns="")
    group.set_goal_orientation_tolerance(0.01)
    group.set_planning_time(5)
    group.allow_replanning(True)
    planning = PlanningSceneInterface("ground_link", ns="")
    planning.clear()
    rospy.sleep(1)
    num_objects = 3 
    rospy.wait_for_service('/clear_octomap')
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap()

    for r in range (0, num_objects):
        planning.clear()
        call_pointcloud_filter_service()
        pnp.wait_for_mesh_and_save()
    	
	# Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        pnp.grasps_received = False
        result = gripper_client_2(-0.5)
        print("Gripper opened")
        selected_grasps = pnp.get_gpd_grasps()
        formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
        successful_grasp = pnp.pick_two_steps(formatted_grasps, verbose=True)
        print ("SUCCESSFUL GRASP IS:", successful_grasp)
        if successful_grasp is not None:
            result = gripper_client_2(0.1)
            print("Gripper closed")
            rospy.sleep(10)
            ## first move ## 
            print("!!!! FIRST MOVE STARTS !!!!")
            move1 = pnp.move_to(successful_grasp.grasp_pose.pose.position.x,
            successful_grasp.grasp_pose.pose.position.y, 0.27,
            successful_grasp.grasp_pose.pose.orientation.x,
            successful_grasp.grasp_pose.pose.orientation.y,
            successful_grasp.grasp_pose.pose.orientation.z,
            successful_grasp.grasp_pose.pose.orientation.w,
            False)
            ## SECOND  MOVE ##
            print("!!!! SECOND MOVE STARTS !!!!") 
            move2 = pnp.move_to(-0.3,0,0.3,successful_grasp.grasp_pose.pose.orientation.x,
            successful_grasp.grasp_pose.pose.orientation.y,
            successful_grasp.grasp_pose.pose.orientation.z,
            successful_grasp.grasp_pose.pose.orientation.w,
            True)

            ## THIRD MOVE ## 
            print("!!!! THIRD MOVE STARTS !!!!") 
            move3 = pnp.move_to(0.065, 0, 0.207, 0, 0, 0, 1, False)
        else:
            print("Grasp NOT performed")


