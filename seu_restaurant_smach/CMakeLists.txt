cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_smach)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  rospy
  smach
  smach_ros
  std_msgs
  tf
  seu_nav
  seu_restaurant_msgs
  seu_voice
)

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs rospy smach smach_ros std_msgs tf seu_nav seu_restaurant_msgs seu_voice
)
