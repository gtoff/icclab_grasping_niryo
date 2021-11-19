# Script files

This directory contains few python scripts we use for demos / utility.
The two most relevant ones are:

- pick_and_place_movegroup.py
- pick_and_place_ppi.py

Usage in simulation:

     python3 pick_and_place_movegroup.py sim

Both scripts will:
- connect to the move group of the arm
- trigger point cloud filtering (segmentation of planar surface vs objects on plane) and clustering
- wait for grasp generation results
- try out all grasps and ask the user whether to perform any succesful one

The difference between the scripts is that the movegroup one uses 2 separated movements for grasping, 
while "ppi" uses the "pick and place" interface that doesn't allow us to visualize the movement before executing it 
