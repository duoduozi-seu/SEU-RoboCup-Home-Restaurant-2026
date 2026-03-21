cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_interaction)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  seu_restaurant_msgs
  seu_restaurant_order_llm
  seu_speech_recognition
  seu_voice
)

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs nav_msgs rospy sensor_msgs std_msgs tf tf2_ros seu_restaurant_msgs seu_restaurant_order_llm seu_speech_recognition seu_voice
)
