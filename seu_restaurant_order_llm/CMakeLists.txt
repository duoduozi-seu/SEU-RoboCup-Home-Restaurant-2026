cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_order_llm)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  seu_restaurant_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib rospy seu_restaurant_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
