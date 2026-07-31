cmake_minimum_required(VERSION 3.0.2)
project(adaptive_amr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs
)

# Message definitions are added in later phases, e.g.:
#   add_message_files(FILES ObjectTrack.msg ObjectTrackArray.msg)
#   generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
