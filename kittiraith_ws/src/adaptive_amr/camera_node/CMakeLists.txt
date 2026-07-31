cmake_minimum_required(VERSION 3.0.2)
project(camera_node)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

catkin_install_python(
  PROGRAMS scripts/camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
