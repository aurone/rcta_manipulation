add_library(pointcloud_occupancy_map_updater pointcloud_occupancy_map_updater.cpp)
target_link_libraries(
    pointcloud_occupancy_map_updater
    ${moveit_ros_planning_LIBRARIES}
    ${message_filters_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${catkin_LIBRARIES})
