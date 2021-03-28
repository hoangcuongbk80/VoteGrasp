cmake_minimum_required(VERSION 2.8.12)
project(force_controllers)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  kdl_parser
  eigen_conversions
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp controller_interface kdl_parser eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS orocos_kdl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(force_controllers src/base_effort_controller.cpp
                              src/hiqp_time_point.cpp
                              src/impedance_controller.cpp
                              src/utilities.cpp)

target_link_libraries(force_controllers ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
