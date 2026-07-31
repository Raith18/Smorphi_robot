cmake_minimum_required(VERSION 3.0.2)
project(time_sync)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs diagnostic_msgs message_filters)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs diagnostic_msgs message_filters
)

catkin_install_python(
  PROGRAMS scripts/time_sync_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
