cmake_minimum_required(VERSION 3.0.2)
project(calibration)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs tf2_ros)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs tf2_ros
)

catkin_install_python(
  PROGRAMS scripts/calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
