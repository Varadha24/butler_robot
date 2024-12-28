cmake_minimum_required(VERSION 3.0.2)
project(butler_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
