cmake_minimum_required(VERSION 2.8.3)
project(marine_radar_tracker)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    grid_map_ros
    marine_sensor_msgs
    pcl_ros
    project11_msgs
    rosbag
    roscpp
    tf2
    tf2_ros
    visualization_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        grid_map_ros
        marine_sensor_msgs
        pcl_ros
        project11_msgs
        roscpp
        tf2
        tf2_ros
        visualization_msgs
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/marine_radar_tracker.cpp)
add_dependencies(${PROJECT_NAME} ${marine_radar_tracker_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(marine_radar_to_pointcloud src/marine_radar_to_pointcloud.cpp)
add_dependencies(marine_radar_to_pointcloud ${marine_radar_tracker_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marine_radar_to_pointcloud ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} marine_radar_to_pointcloud
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
