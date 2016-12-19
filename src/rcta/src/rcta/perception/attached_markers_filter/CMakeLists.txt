add_executable(attached_markers_filter attached_markers_filter.cpp)
target_link_libraries(
    attached_markers_filter
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    hdt_robot_model)

