cmake_minimum_required(VERSION 3.0.2)
project(seu_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  rospy
  std_msgs
  message_generation
)

# 生成服务文件
add_service_files(
  FILES
  seu_nav.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装 Python 脚本
# catkin_install_python(PROGRAMS
#   scripts/nav_service.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
