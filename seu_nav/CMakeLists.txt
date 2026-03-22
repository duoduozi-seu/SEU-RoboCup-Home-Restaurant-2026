cmake_minimum_required(VERSION 3.0.2)
project(seu_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  seu_restaurant_msgs
  std_msgs
)

add_service_files(
  FILES
  seu_nav.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime nav_msgs rospy seu_restaurant_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
