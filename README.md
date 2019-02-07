# icclab_grasping_niryo
Scripts, configurations, launch files for grasping with Niryo arms at ICCLab

### Setting up the Realsense D415 camera
1. Connect the camera to your machine and follow the instructions found here https://github.com/intel-ros/realsense
2. If the camera is not recognized follow the instructions in **notes_realsense.txt**

### Configuring connection between your PC and robot
1. Get the IP addresses of your PC and niryo arm using **ifconfig** (optionally you can set up aliases for these IPs in /etc/hosts)
2. Connect to the Niryo arm using **ssh niryo@<IP_OF_NIRYO>** - password: **robotics** and then run **export ROS_HOSTNAME=<IP_OF_NIRYO>**
3. On your PC run **export ROS_MASTER_URI=http://<IP_OF_NIRYO>:11311** and **export ROS_HOSTNAME=<IP_OF_PC>**

### Tool dependencies
1. Aruco tracker ROS package - http://wiki.ros.org/ar_track_alvar
2. Find object 2d - http://wiki.ros.org/find_object_2d

### Launch the grasping demo
1. On a terminal run **roslaunch icclab_grasping_niryo niryo_one_aruco_single_view.launch**
2. Bring up the Find Object GUI and specify the object you wish to grasp
3. On a new terminal run the **niryo_demo.py** file in *icclab_grasping_niryo/scripts* (you'll have to export the ROS network 
   variables again before running the script)
