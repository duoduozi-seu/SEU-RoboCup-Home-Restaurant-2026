cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  FindWavingCustomer.action
  AlignAndTakeOrder.action
  RestaurantNav.action
  ParseRestaurantOrder.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs
)
