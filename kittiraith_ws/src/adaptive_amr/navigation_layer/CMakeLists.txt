cmake_minimum_required(VERSION 3.0.2)
project(navigation_layer)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
)

# Expose src/navigation_layer as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs tf2_ros
                diagnostic_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/navigation_layer_node.py
    scripts/behavior_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
