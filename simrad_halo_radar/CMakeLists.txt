cmake_minimum_required(VERSION 3.8)
project(simrad_halo_radar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
    marine_radar_control_msgs
    marine_sensor_msgs
    rosbag
    roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
   CATKIN_DEPENDS marine_radar_control_msgs marine_sensor_msgs rosbag
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (SOURCES
    src/halo_radar/halo_radar.cpp
    src/halo_radar/halo_radar_node.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(fix_angle_increments src/halo_radar/fix_angle_increments.cpp)
add_dependencies(fix_angle_increments ${catkin_EXPORTED_TARGETS})
target_link_libraries(fix_angle_increments ${catkin_LIBRARIES})

add_executable(fix_scan_times src/halo_radar/fix_scan_times.cpp)
add_dependencies(fix_scan_times ${catkin_EXPORTED_TARGETS})
target_link_libraries(fix_scan_times ${catkin_LIBRARIES})
