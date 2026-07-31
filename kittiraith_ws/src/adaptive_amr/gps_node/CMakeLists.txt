cmake_minimum_required(VERSION 3.0.2)
project(gps_node)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

catkin_install_python(
  PROGRAMS scripts/gps_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
