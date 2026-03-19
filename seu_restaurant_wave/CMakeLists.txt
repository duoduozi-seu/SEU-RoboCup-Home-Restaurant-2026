cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_wave)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  seu_restaurant_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs nav_msgs rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros seu_restaurant_msgs
)
