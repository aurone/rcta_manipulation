add_executable(costmap_object_detection costmap_object_detection.cpp)
target_link_libraries(
    costmap_object_detection
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    msg_utils)

