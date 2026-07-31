cmake_minimum_required(VERSION 3.0.2)
project(adaptive_amr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs
)

add_message_files(
  FILES
  ObjectDetection.msg
  ObjectDetectionArray.msg
  ObjectTrack.msg
  ObjectTrackArray.msg
  DynamicObstacle.msg
  DynamicObstacleArray.msg
)

add_service_files(
  FILES
  Relocalize.srv
  SetGoal.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
