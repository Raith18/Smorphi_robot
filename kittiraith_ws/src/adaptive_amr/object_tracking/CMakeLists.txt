cmake_minimum_required(VERSION 3.0.2)
project(object_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs diagnostic_msgs visualization_msgs adaptive_amr_msgs
)

# Expose src/object_tracking as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs diagnostic_msgs
                visualization_msgs adaptive_amr_msgs
)

catkin_install_python(
  PROGRAMS scripts/object_tracking_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
