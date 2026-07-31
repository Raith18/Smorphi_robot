cmake_minimum_required(VERSION 3.0.2)
project(evaluation)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs diagnostic_msgs)

# Expose src/evaluation as an importable Python module (pure math metrics).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs diagnostic_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/benchmark.py
    scripts/profile.py
    scripts/perf_collector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
