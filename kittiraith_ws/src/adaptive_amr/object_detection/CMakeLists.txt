cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs tf2_ros message_filters
  diagnostic_msgs adaptive_amr_msgs
)

# Expose src/object_detection as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf2_ros
                message_filters diagnostic_msgs adaptive_amr_msgs
)

catkin_install_python(
  PROGRAMS scripts/object_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
