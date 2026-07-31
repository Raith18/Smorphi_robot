cmake_minimum_required(VERSION 3.0.2)
project(semantic_segmentation)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs message_filters diagnostic_msgs
)

# Expose src/semantic_segmentation as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_filters diagnostic_msgs
)

catkin_install_python(
  PROGRAMS scripts/semantic_segmentation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
