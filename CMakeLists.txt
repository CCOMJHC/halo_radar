cmake_minimum_required(VERSION 3.0.2)
project(marine_radar_control_msgs)

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
