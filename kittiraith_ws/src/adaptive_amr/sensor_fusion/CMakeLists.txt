cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
  message_filters diagnostic_msgs
)

# Expose src/sensor_fusion as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf2_ros
                tf2_geometry_msgs message_filters diagnostic_msgs
)

catkin_install_python(
  PROGRAMS scripts/sensor_fusion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
