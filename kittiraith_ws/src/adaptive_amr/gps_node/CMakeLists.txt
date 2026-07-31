cmake_minimum_required(VERSION 3.0.2)
project(gps_node)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
