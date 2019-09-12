# icclab_grasping_niryo
Scripts, configurations, launch files for grasping with Niryo arms at ICCLab

### Setup
1. Connect the camera
2. cd rosdocked_irlab/workspace_inccluded
3. ./run_with_dev.sh
4. Setup connection (see below)

### Setting up the Realsense D415 camera
1. Connect the camera to your machine and follow the instructions found here https://github.com/intel-ros/realsense
2. If the camera is not recognized follow the instructions in **notes_realsense.txt**

### Configuring connection between your PC and robot
1. Get the IP addresses of your PC and niryo arm using **ifconfig** (optionally you can set up aliases for these IPs in /etc/hosts and /etc/hostname)
2. Connect to the Niryo arm using **ssh niryo@<IP_OF_NIRYO>** - password: **robotics** and then run **export ROS_IP=<IP_OF_NIRYO>**
3. On your PC run **export ROS_MASTER_URI=http://<IP_OF_NIRYO>:11311** and **export ROS_IP=<IP_OF_PC>**

### Tool dependencies
1. Aruco tracker ROS package - http://wiki.ros.org/ar_track_alvar
2. Find object 2d - http://wiki.ros.org/find_object_2d

### Calibrating the camera pose - Doesn't work for now
1. Run **roslaunch icclab_grasping_niryo camera_niryo.launch**
2. Setup the markers on the table 
3. Run **python calibrate_camera.py**
This should update the **camera_params.yaml** file with the camera_depth_optical_frame -> ground_link transform in the form
[position.x 
 position.y
 position.z
 orientation.x
 orientation.y
 orientation.z
 orientation.w]
This process need only be done every time the camera's pose is altered with respect to the robot.

### GPD Demo
1. roslaunch icclab_grasping_niryo niryo_one_aruco_view_gpd.launch
2. python ~/catkin_ws/src/icclab_grasping_niryo/scripts/pick_and_place_niryo.py

### Launch the grasping demo
1. On a terminal run **roslaunch icclab_grasping_niryo niryo_one_aruco_single_view.launch**
2. Bring up the Find Object GUI and specify the object you wish to grasp
3. On a new terminal run the **niryo_demo.py** file in *icclab_grasping_niryo/scripts* (you'll have to export the ROS network 
   variables again before running the script)
