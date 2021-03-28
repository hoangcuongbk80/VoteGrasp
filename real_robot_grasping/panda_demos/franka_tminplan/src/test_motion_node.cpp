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

#include <franka/exception.h>
#include <franka/robot.h>

#include "traj_planning.h"

double grasp[8]; // x,y,z,rx,ry,rz,ree,width
float width = 0.06;
std::vector<double> initial_pose;

void move_robot()
{
  try 
  {
    franka::Robot robot("172.16.0.103");

    //setDefaultBehavior(robot);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});

    //MotionGenerator motion_generator(speed_factor, q_goal);
    std::array<double, 7> q_init = {0, 0, 0, -1, 0, 1, 0};
    std::array<double, 7> q_goal = {0, 0, 0, -1.7, 0, 1.7, 0};
    std::array<double, 7> dq_init = {0, 0, 0, 0, 0, 0, 0};
    double speed_factor = 0.05;

    for (size_t i = 0; i < 7; i++) {q_goal[i] = grasp[i];}

    TrajPlanning traj_planning(speed_factor,q_init,q_goal,dq_init);

    // real motion
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(traj_planning);    // operator()
    std::cout << "Motion finished" << std::endl;
  } 
  catch (const franka::Exception& e) 
  {
    franka::Robot robot("172.16.0.103");

    std::cout << robot.readOnce() << std::endl;
    std::cout << e.what() << std::endl;
  }
  std::cerr << "\nFinish move!\n";
  return;
}

void initialize_pos()
{
  for (size_t i = 0; i < 7; i++) {grasp[i]=initial_pose[i];}
  move_robot();
}

void move_to_place()
{
  move_robot();
}

void move_to_pick(const std_msgs::Float64MultiArray::ConstPtr& array)
{
  for (size_t i = 0; i < 8; i++)
  {
    grasp[i] = (double) array->data[i];
  }
	move_robot();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasping_robotlab_node");
  ros::NodeHandle n("~");
  std::string robot_ip="172.16.0.103";
  n.getParam("robot_ip", robot_ip);
  n.getParam("initial_pose", initial_pose);
  initialize_pos();
  
  ros::NodeHandle nh_;
  ros::Subscriber grasp =  nh_.subscribe<std_msgs::Float64MultiArray>("/votegrasp/grasp", 1, move_to_pick);
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

