// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */

int main(int argc, char** argv) {

  try {
    franka::Robot robot("172.16.0.103");
    franka::RobotState robot_state = robot.readOnce();

    std::cout << "Joint position: " 
        << robot_state.q_d[0] << ", "
        << robot_state.q_d[1] << ", "
        << robot_state.q_d[2] << ", "
        << robot_state.q_d[3] << ", "
        << robot_state.q_d[4] << ", "
        << robot_state.q_d[5] << ", "
        << robot_state.q_d[6]
        << std::endl;
    std::cout << "Cartesian position: " 
        << robot_state.O_T_EE[12] << ", "
        << robot_state.O_T_EE[13] << ", "
        << robot_state.O_T_EE[14]
        << std::endl;

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
