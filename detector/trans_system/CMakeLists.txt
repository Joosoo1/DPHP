cmake_minimum_required(VERSION 3.0.2)
project(trans_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs
)

# 安装Python脚本
catkin_install_python(PROGRAMS
  door_position_transformer_optimized.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装配置文件
install(FILES
  calib.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
