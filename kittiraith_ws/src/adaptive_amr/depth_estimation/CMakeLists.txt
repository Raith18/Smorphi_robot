cmake_minimum_required(VERSION 3.0.2)
project(depth_estimation)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs diagnostic_msgs
)

# Expose src/depth_estimation as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs diagnostic_msgs
)

catkin_install_python(
  PROGRAMS scripts/depth_estimation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
