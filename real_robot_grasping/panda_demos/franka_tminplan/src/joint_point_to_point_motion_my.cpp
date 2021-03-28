#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "traj_planning.h"

// #include <Poco/DateTimeFormatter.h>
// #include <Poco/File.h>
// #include <Poco/Path.h>
// #include <fstream>
// #include <iomanip>


// void writeLogToFile(const std::vector<franka::Record>& log) {
//   if (log.empty()) {
//     return;
//   }
//   try {
//     Poco::Path temp_dir_path(Poco::Path::temp());
//     temp_dir_path.pushDirectory("libfranka-logs");
//     Poco::File temp_dir(temp_dir_path);
//     temp_dir.createDirectories();
//     std::string now_string =
//         Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
//     std::string filename = std::string{"log-" + now_string + ".csv"};
//     Poco::File log_file(Poco::Path(temp_dir_path, filename));
//     if (!log_file.createFile()) {
//       std::cout << "Failed to write log file." << std::endl;
//       return;
//     }
//     std::ofstream log_stream(log_file.path().c_str());
//     log_stream << franka::logToCSV(log);
//     std::cout << "Log file written to: " << log_file.path() << std::endl;
//   } catch (...) {
//     std::cout << "Failed to write log file." << std::endl;
//   }
// }
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_point_to_point_motion_my");
  ros::NodeHandle n("~");

  //  if (argc != 10) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
  //             << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
  //             << "<speed-factor>" << std::endl
  //             << "joint0 to joint6 are joint angles in [rad]." << std::endl
  //             << "speed-factor must be between zero and one." << std::endl;
  //   return -1;
  // }
    
    try {
    franka::Robot robot("172.16.0.103");

    //setDefaultBehavior(robot);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});


    // std::array<double, 7> q_goal;
    // for (size_t i = 0; i < 7; i++) {
    //   q_goal[i] = std::stod(argv[i + 2]);
    // }
    // double speed_factor = std::stod(argv[9]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    // robot.setCollisionBehavior(
    //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});

    //MotionGenerator motion_generator(speed_factor, q_goal);
    std::array<double, 7> q_init = {0, 0, 0, -1, 0, 1, 0};
    std::array<double, 7> q_goal = {0, 0, 0, -1.7, 0, 1.7, 0};
    std::array<double, 7> dq_init = {0, 0, 0, 0, 0, 0, 0};
    double speed_factor;

    n.getParam("speed_factor", speed_factor);

    std::vector<double> para_goal;
    n.getParam("para_goal", para_goal);
    for (size_t i = 0; i < 7; i++) {q_goal[i] = para_goal[i];}

    TrajPlanning traj_planning(speed_factor,q_init,q_goal,dq_init);

    // // visualization
    // traj_planning.initTminTraj();
    // feasible_traj = traj_planning.calculateTminTraj();
    // /*Real-time trajectory generation and publish*/
    // if(feasible_traj) {
    //   ros::Rate rate(publish_rate);
    //   sensor_msgs::JointState states;
    //   states.effort.resize(joint_names.size());
    //   states.name.resize(joint_names.size());
    //   states.position.resize(joint_names.size());
    //   states.velocity.resize(joint_names.size());
    //   for (size_t i = 0; i < joint_names.size(); i++) {
    //     states.name[i] = joint_names[i];
    //   }
    //   ros::Publisher publisher = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
    //   bool motion_finished = false;
    //   while ((ros::ok()) && (!motion_finished)) {
    //     states.header.stamp = ros::Time::now();
    //     motion_finished=traj_planning.generateMotion();
    //     for (size_t i = 0; i < joint_names.size(); i++) {
    //         states.position[i] = traj_planning.q_current[i];
    //     }
    //     std::cout << "Joint position: " 
    //         << states.position[0] << ", "
    //         << states.position[1] << ", "
    //         << states.position[2] << ", "
    //         << states.position[3] << ", "
    //         << states.position[4] << ", "
    //         << states.position[5] << ", "
    //         << states.position[6]
    //         << std::endl;

    //         publisher.publish(states);
    //         ros::spinOnce();
    //         rate.sleep();
    //  }
    // } else {
    //   std::cout << "Infeasible trajetory!" << std::endl;
    // }

    // real motion
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(traj_planning);    // operator()
    std::cout << "Motion finished" << std::endl;

    // std::cout << "Time minimal trajectory parameter: " 
    //     << traj_planning.traj_para[0] << ", "
    //     << traj_planning.traj_para[1] << ", "
    //     << traj_planning.traj_para[2] << ", "
    //     << traj_planning.traj_para[3] << ", "
    //     << traj_planning.traj_para[4] << ", "
    //     << traj_planning.traj_para[5] << ", "
    //     << traj_planning.traj_para[6] << ", "
    //     << traj_planning.traj_para[7]
    //     << std::endl;

  } catch (const franka::Exception& e) {
    franka::Robot robot("172.16.0.103");

    std::cout << robot.readOnce() << std::endl;
    std::cout << e.what() << std::endl;

    

    return -1;
  }


  return 0;
}

