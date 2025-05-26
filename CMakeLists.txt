cmake_minimum_required(VERSION 3.0.2)
project(motion_tube_planner)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_tube_planner
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller
  src/controller_node.cpp
  src/controller.cpp
)
target_link_libraries(controller
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
