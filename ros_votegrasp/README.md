The tools require full ROS installation. The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic]

1. Build
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make --pkg ros_votegrasp

2. Operate
   ```bash
   $ cd ~/catkin_ws
   $ roslaunch ros_votegrasp model2scene.launch
   $ roslaunch ros_votegrasp depth2pointcloud.launch
   $ roslaunch ros_votegrasp visualize_pointcloud.launch
   $ roslaunch ros_votegrasp visualize_grasp.launch

<img src="doc/grasp_pc_rviz.png" width="800" />

<img src="doc/grasp_model_rviz.png" width="800" />
