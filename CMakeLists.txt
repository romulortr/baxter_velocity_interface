cmake_minimum_required(VERSION 2.8.3)
project(baxter_velocity_interface)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  baxter_core_msgs
  baxter_interface
)


catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  baxter_interface
  baxter_core_msgs
)
