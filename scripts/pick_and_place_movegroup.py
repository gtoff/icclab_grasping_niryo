#!/usr/bin/env python3
import sys, getopt, signal
sys.path.append('/home/ros/catkin_ws/src/icclab_grasping_niryo/scripts/')
import rospy
import numpy as np
import copy
import tf
import vtk
from tf import TransformListener
import tf2_ros
import yaml
import datetime
import gc
#from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd_ros.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3, Pose, TransformStamped, PointStamped, Vector3Stamped
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
import sensor_msgs.msg  # import PointCloud2
#from gpd_controller import GpdGrasps
#from robot_controller import RobotPreparation
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from tf.transformations import *
from filter_pointcloud_client import call_pointcloud_filter_service
from cluster_pointcloud_client import call_pointcloud_cluster_service
from send_gripper import gripper_client_2
from rosnode import get_node_names, kill_nodes
from sensor_msgs import point_cloud2
import math
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
from niryo_one_python_api.niryo_one_api import *
import time
import subprocess
from std_srvs.srv import Empty

camera_frame_id = "camera_color_optical_frame"  #default to simulation, but is required to be set sim/hw
simulation = True #default to simulation, but is required to be set sim/hw

class RAPPickNPlace(object):
    if not simulation:
        niryo = None # the interface to physical arm
    movegroup = None # the interface to MoveGroupCommander
    planningscene = None # the planning scene
    pose_publisher = None # used to visualize grasp pose in rviz
    pre_grasp_offset = -0.1
    grasp_offset = -0.03
    grasps_received = False


    def __init__(self):
        rospy.init_node("RAPPickNPlace")
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList,
                                                 self.grasp_callback)
        self.pose_publisher = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1)

        if not simulation:
            # Start physical arm
            print("--- Start Physical Arm ---")
            self.niryo = NiryoOne()
            print("Make sure calibration is already performed on arm !")
            time.sleep(1)
            # Set up gripper 1
            self.niryo.change_tool(TOOL_GRIPPER_1_ID)

        ## Initialize MoveGroupCommander (movegroup), PlanningScene (planningscene)
        group_name = "arm"
        self.movegroup = moveit_commander.MoveGroupCommander(group_name, robot_description="/robot_description", ns="")
        self.movegroup.set_goal_orientation_tolerance(0.01)
        self.movegroup.set_goal_tolerance(0.01)
        self.movegroup.set_planning_time(5)
        self.movegroup.allow_replanning(True)
        self.planningscene = PlanningSceneInterface(camera_frame_id) 
        self.tf = tf.TransformListener()

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasps_received = True
        if (len(msg.grasps)==0):
            print("No grasps found, aborting!")
        else:
            print("Received new grasps")

    def show_grasp_pose(self, grasp_pose):
         self.pose_publisher.publish(grasp_pose)

    def get_gpd_grasps(self):
        print("Waiting for grasps to arrive")
        while (self.grasps_received == False):
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_poses(self, selected_grasps):
        grasp_poses = []
        pre_grasp_poses = []
        cont = 0
        filtered_orientation = 0
        self.tf.waitForTransform(camera_frame_id, '/base_link', rospy.Time(), rospy.Duration(2.0))
        for i in range(0, len(selected_grasps)):

            # create pose
            quat = self.trans_matrix_to_quaternion(selected_grasps[i])
            gp = PoseStamped()
            gp.header.frame_id = camera_frame_id
            gp.pose.position.x = selected_grasps[i].position.x + self.grasp_offset * selected_grasps[i].approach.x
            gp.pose.position.y = selected_grasps[i].position.y + self.grasp_offset * selected_grasps[i].approach.y
            gp.pose.position.z = selected_grasps[i].position.z + self.grasp_offset * selected_grasps[i].approach.z
            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = float(quat.elements[0])
       
            translated_pose = self.tf.transformPose("ground_link", gp)

            z_axis_unit = (0, 0, 1)
            approach_vector = Vector3Stamped()
            approach_vector.header.frame_id = camera_frame_id
            approach_vector.vector.x = selected_grasps[i].approach.x
            approach_vector.vector.y = selected_grasps[i].approach.y
            approach_vector.vector.z = selected_grasps[i].approach.z
            approach_vector_in_base_frame = self.tf.transformVector3("ground_link", approach_vector)
            dot_product = numpy.dot(z_axis_unit, [approach_vector_in_base_frame.vector.x,
                                                 approach_vector_in_base_frame.vector.y,
                                                 approach_vector_in_base_frame.vector.z ])
            if (dot_product >= 0.5):
            #     # filter it out
                filtered_orientation += 1
                print(repr(filtered_orientation) + " Grasp filtered because not coming from above")
                continue

            grasp_poses.append(translated_pose)

            pre_gp = copy.deepcopy(gp)
            pre_gp.pose.position.x = selected_grasps[i].position.x + self.pre_grasp_offset * selected_grasps[i].approach.x
            pre_gp.pose.position.y = selected_grasps[i].position.y + self.pre_grasp_offset * selected_grasps[i].approach.y
            pre_gp.pose.position.z = selected_grasps[i].position.z + self.pre_grasp_offset * selected_grasps[i].approach.z
            translated_pose = self.tf.transformPose("ground_link", pre_gp)
            pre_grasp_poses.append(translated_pose)

        # sort both lists trying higher grasps first
        pre_grasp_poses, grasp_poses = zip(*sorted(zip(pre_grasp_poses, grasp_poses), key=lambda pair: pair[0].pose.position.z, reverse=True))
        
        return pre_grasp_poses, grasp_poses

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                      [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                      [grasp.approach.z, grasp.binormal.z, grasp.axis.z]])
        return Quaternion(matrix=r)

    def add_object_mesh(self):
        obj_pose = Pose()
        obj_pose.position.x = 0
        obj_pose.position.y = 0
        obj_pose.position.z = 0
        obj_pose.orientation.x = 0
        obj_pose.orientation.y = 0
        obj_pose.orientation.z = 0
        obj_pose.orientation.w = 1
        self.planningscene.addMesh("object", obj_pose, "object.stl", use_service=True)

    def wait_for_mesh_and_save(self):
        print("Subscribing to pointcloud to generate mesh")
        self.obj_pc_subscriber = rospy.Subscriber("/cloud_indexed_pc_only", sensor_msgs.msg.PointCloud2,
                                                  self.obj_pointcloud_callback)

    def obj_pointcloud_callback(self, msg):  # msg is a sensor_msgs.msg.PointCloud2
        print("Pointcloud received")
        cloud = []
        for p in point_cloud2.read_points(msg, skip_nans=True):
            cloud.append([p[0], p[1], p[2]])
        self.create_mesh_and_save(cloud)
        print("Mesh generated")
        self.obj_pc_subscriber.unregister()

    def create_mesh_and_save(self, cloud):
        filename = "object.stl"

        vtk_points = vtk.vtkPoints()
        np_cloud = np.asarray(cloud)

        for i in range(0, np_cloud.shape[0]):
            vtk_points.InsertPoint(i, np_cloud[i][0], np_cloud[i][1], np_cloud[i][2])

        profile = vtk.vtkPolyData()
        profile.SetPoints(vtk_points)

        delny = vtk.vtkDelaunay2D()
        delny.SetInputData(profile)
        delny.SetTolerance(0.001)

        stl_writer = vtk.vtkSTLWriter()
        stl_writer.SetFileName(filename)
        stl_writer.SetFileName(filename)
        stl_writer.SetInputConnection(delny.GetOutputPort())
        stl_writer.Write()

    def get_grasp_poses_and_mesh(self):
        self.planningscene.clear()
        rospy.sleep(1)
        rospy.wait_for_service('/clear_octomap')
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
        call_pointcloud_filter_service()
#        call_pointcloud_cluster_service()
        self.wait_for_mesh_and_save()

        # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        selected_grasps = self.get_gpd_grasps()
        pre_grasp_poses, grasp_poses = self.generate_grasp_poses(selected_grasps)
        return pre_grasp_poses, grasp_poses

    
    def plan_and_move_to_pregrasp(self, grasp_pose, move=False):
        """
        Plans and executes the movement to a grasp pose
        Parameters
        ----------
        grasp_poses : PoseStamped[]
            An array of pose stamped objects generated by the GPD algorithm
            They need to be adjusted in a pre-grasp to approach the object in the direction
            of the gripper fingers (e.g., 5 cm "away" from the object if the fingers are less the 5cm long)
        """
        
        # publish grasp for rviz visualization
        self.show_grasp_pose(grasp_pose)
        self.movegroup.set_start_state_to_current_state()
        self.movegroup.set_pose_target(grasp_pose.pose)
        plan_success, plan, planning_time, error_code = self.movegroup.plan()

        if (not move):
            return plan_success
        elif (len(plan.joint_trajectory.points) != 0):
            inp = input("Have a look at the planned motion. Do you want to proceed? y/n/retry/exit: ")
            if (inp == 'y'):
                print("Executing grasp: ")
                pick_result = self.movegroup.execute(plan, wait=True)
                if pick_result == True:
                    self.movegroup.stop()
                    self.movegroup.clear_pose_targets()
                    self.movegroup.set_start_state_to_current_state()
                    return True
                else:
                    self.movegroup.stop()
                    self.movegroup.clear_pose_targets()
            elif (inp == 'retry'):
                return self.plan_and_move_to_pregrasp(grasp_pose, move=True)
            elif (inp == 'exit'):
                self.movegroup.stop()
                self.movegroup.clear_pose_targets()
                exit(1)
    

    def plan_and_move_to_grasp(self, grasp_pose, move=False):
        # publish grasp for rviz visualization
        self.show_grasp_pose(grasp_pose)
        print("Planning end grasp")
        return self.plan_and_move_to_pregrasp(grasp_pose, move)

    def attach_graspedobject_to_arm(self):
        attach_link = "gripper_base_link"
        touch_links = ["gripper_base_link","gripper_left_finger_base_link","gripper_left_finger_link","gripper_right_finger_base_link","gripper_right_finger_link","gripper_servo_link"]
        self.movegroup.attach_object("object", attach_link, touch_links)
        print("object attached to arm")

    def move_to(self, x, y, z, qx, qy, qz, qw, drop):
        print("Dropping object on robot")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        self.movegroup.set_start_state_to_current_state()
        self.movegroup.set_goal_tolerance(0.05)
        self.movegroup.set_pose_target(pose_goal)

        plan_success, plan, planning_time, error_code = self.movegroup.plan()
        rospy.sleep(1)
        cont_plan_drop = 0
        while ((len(plan.joint_trajectory.points) == 0) and (cont_plan_drop < 10)):
            plan_success, plan, planning_time, error_code = self.movegroup.plan()
            rospy.sleep(1)
            cont_plan_drop += 1
        if (len(plan.joint_trajectory.points) != 0):
            inp = input("Have a look at the planned motion. Do you want to proceed? y/n/retry/exit: ")[0]
            if (inp == 'y'):
                print("Executing dropping: ")
                result = self.movegroup.execute(plan, wait=True)
                rospy.sleep(1)
                if (drop==True):
                    if not simulation:
                        self.niryo.open_gripper(TOOL_GRIPPER_1_ID, 200)
                    else:
                        result = gripper_client_2(-0.5)
                    rospy.sleep(3)
                    print("Gripper opened")
                    print("Dropping successful!")
                    self.movegroup.detach_object("object")
                    self.movegroup.stop()
                    self.movegroup.clear_pose_targets()
            elif (inp == 'exit'):
                self.movegroup.stop()
                self.movegroup.clear_pose_targets()
                place_successful = False
                exit(1)

    def place_in_box(self, successful_grasp_pose):
        if not simulation:
            self.niryo.close_gripper(TOOL_GRIPPER_1_ID, 200)
        else:
            result = gripper_client_2(0.1)
        print("gripper closed")
        rospy.sleep(1)
        ## first move ##
        print("!!!! FIRST MOVE STARTS !!!!")
        move1 = self.move_to(successful_grasp_pose.pose.position.x,
                            successful_grasp_pose.pose.position.y, 0.27,
                            successful_grasp_pose.pose.orientation.x,
                            successful_grasp_pose.pose.orientation.y,
                            successful_grasp_pose.pose.orientation.z,
                            successful_grasp_pose.pose.orientation.w,
                            False)
        ## SECOND  MOVE ##
        print("!!!! SECOND MOVE STARTS !!!!")
        move2 = self.move_to(-0.3, 0, 0.3, successful_grasp_pose.pose.orientation.x,
                            successful_grasp_pose.pose.orientation.y,
                            successful_grasp_pose.pose.orientation.z,
                            successful_grasp_pose.pose.orientation.w,
                            True)
        ## THIRD MOVE ##
        print("!!!! THIRD MOVE STARTS !!!!")
        move3 = self.move_to(0.065, 0, 0.207, 0, 0, 0, 1, False)


    def run(self):
        start_time = datetime.datetime.now()
        
        # Request point cloud segmentation, mesh and grasps generation
        # Grasps will contain and array of PoseStamped
        pre_grasp_poses, grasp_poses = self.get_grasp_poses_and_mesh()

        # add mesh to planning scene
        self.add_object_mesh()

        # open gripper
        if not simulation:
            self.niryo.open_gripper(TOOL_GRIPPER_1_ID, 200)
        else:
            result = gripper_client_2(-0.8)
        print("Gripper opened")

        i = 0
        grasp_successful = False
        #TODO
        # Create a loop of pregrasps and grasps to successfully grasp the object, see below the invocation of a pre_grasp
        while (i < len(pre_grasp_poses) and not(grasp_successful)):
            # plan end grasp
            ok = self.plan_and_move_to_grasp(grasp_poses[i], move=False)
            if (ok):
                # plan and move to pre-grasp
                print("Found successful end grasp")
                self.plan_and_move_to_pregrasp(pre_grasp_poses[i], move=True)
                #TODO
                # Plan move to grasp from the reached point and get a boolean value if succesful
                grasp_successful = self.plan_and_move_to_grasp(grasp_poses[i], move=True)
            i+=1

        if (grasp_successful):
            # if successful: attach to arm using planning scene
            # the object is already know to the planning scene with name "object"
            self.attach_graspedobject_to_arm()

            # close gripper
            if not simulation:
               self.niryo.close_gripper(TOOL_GRIPPER_1_ID, 200)
            else:
               result = gripper_client_2(0.1)
            print("Gripper 1 closed")

               # plan and move for place
             # TODO a function that moves the arm somewhere else to release the object and detach it from the planning scene
            result = self.place_in_box(grasp_poses[i-1])

        print("Demo runtime: " + str(datetime.datetime.now() - start_time))
        return


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    pnp = RAPPickNPlace()
    pnp.run()

"""     if len(sys.argv) > 1:
        if (sys.argv[1] == "sim"):
            print("You selected simulation")
            camera_frame_id = "camera_color_optical_frame"
            simulation=True
        elif (sys.argv[1] == "hw"):
            print("You selected hw")
            camera_frame_id = "camera_color_optical_frame"
            simulation=False
        else:
            print("ERROR: Choose either sim or hw option")
            sys.exit()
    else:
        print("ERROR: Set sim/hw option!")
        sys.exit() """


    

