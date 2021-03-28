cmake_minimum_required(VERSION 2.8.3)
project(move_franka)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/move_franka_node.cpp)
add_executable(execute_grasp_node src/execute_grasp_node.cpp)
add_executable(test_robot_node src/test_robot_node.cpp)


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARIES})
target_link_libraries(execute_grasp_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARIES})
target_link_libraries(test_robot_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARIES})