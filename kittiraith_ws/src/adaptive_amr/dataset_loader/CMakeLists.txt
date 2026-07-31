cmake_minimum_required(VERSION 3.0.2)
project(dataset_loader)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs tf2_msgs rosbag
)

# Expose src/dataset_loader as an importable Python module for other packages.
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf2_msgs rosbag
)

catkin_install_python(
  PROGRAMS scripts/kitti_to_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
