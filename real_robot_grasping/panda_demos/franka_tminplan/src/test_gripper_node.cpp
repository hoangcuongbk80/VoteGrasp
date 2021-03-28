#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka_tminplan/test_gripper.h>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "gripper_publisher");

  ros::NodeHandle n("~");
  double width, epsilon_inner, epsilon_outer, speed, force;
  n.getParam("epsilon_inner", epsilon_inner);
  n.getParam("epsilon_outer", epsilon_outer);
  n.getParam("speed", speed);
  n.getParam("force", force);
  n.getParam("width", width);

  ros::NodeHandle sunda_n;

  ros::Publisher gripper_move_goal = sunda_n.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal",1);
  ros::Publisher gripper_homing_goal = sunda_n.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal",1);
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
    gripper_homing_goal.publish(HAClient_);
 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
