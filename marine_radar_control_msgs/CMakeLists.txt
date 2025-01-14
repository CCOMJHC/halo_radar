cmake_minimum_required(VERSION 3.8)
project(marine_radar_control_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
  FILES
  RadarControlItem.msg
  RadarControlSet.msg
  RadarControlValue.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)
