cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_order_llm)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  ParseRestaurantOrder.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
