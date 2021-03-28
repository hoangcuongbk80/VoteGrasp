# ros_votegrasp

## Introduction
This repository contains code for data processing in ground-truth data creation stage and visualization for VoteGrasp.

## Installation
The tools require full ROS installation. The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic]
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make install
   ```
## Visualization
### To visualize 3D point cloud:
   ```bash
   $ roslaunch ros_votegrasp visualize_pointcloud.launch
   ```
### To visualize grasp:
   ```bash
   $ roslaunch ros_votegrasp visualize_grasp.launch
   ```
  
## Data processing in ground-truth data creation

### To convert depth images from blender to 3D point clouds
   ```bash
   $ roslaunch ros_votegrasp depth2pointcloud.launch
   ```
### To map grasps to scene point clouds
   ```bash
   $ roslaunch ros_votegrasp model2scene.launch
   ```
