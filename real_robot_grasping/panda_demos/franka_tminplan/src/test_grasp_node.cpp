#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_tminplan/test_gripper.h>

#include "traj_planning.h"

float width = 0.06;

void getwidth(const std_msgs::Float64::ConstPtr& wid)
{
   width = wid->data;  
   std::cerr << "Grasp width: " << width << "\n"; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasping_robotlab_node");
  ros::NodeHandle n("~");
  double epsilon_inner, epsilon_outer, speed, force;
  n.getParam("epsilon_inner", epsilon_inner);
  n.getParam("epsilon_outer", epsilon_outer);
  n.getParam("speed", speed);
  n.getParam("force", force);
  n.getParam("width", width);
  
  ros::NodeHandle nh_;
  ros::Subscriber gripper_move_width =  nh_.subscribe<std_msgs::Float64>("/votegrasp/width", 1, getwidth);
  
  ros::NodeHandle grasp_n;
  ros::Publisher gripper_goal = grasp_n.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal",1);
  franka_gripper::GraspActionGoal MAClient_;
  franka_gripper::MoveActionGoal HAClient_;
  MAClient_.goal.epsilon.inner = epsilon_inner;
  MAClient_.goal.epsilon.outer = epsilon_outer;
  MAClient_.goal.speed = speed;
  MAClient_.goal.force = force;
  HAClient_.goal.speed = speed;

  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    HAClient_.goal.width = width;
    gripper_goal.publish(HAClient_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

