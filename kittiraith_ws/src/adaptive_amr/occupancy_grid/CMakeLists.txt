cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros
  diagnostic_msgs message_filters
)

# Expose src/occupancy_grid as an importable Python module (pure math).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs
                tf2_ros diagnostic_msgs message_filters
)

catkin_install_python(
  PROGRAMS scripts/occupancy_grid_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
