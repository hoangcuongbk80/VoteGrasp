#pragma once
#include <memory>
#include <string>
#include "ros/ros.h"
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
//#include <franka_msgs/FrankaModel.h>
#include <realtime_tools/realtime_publisher.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka/exception.h>
#include <franka/gripper.h>

double g_width;
double xx;

