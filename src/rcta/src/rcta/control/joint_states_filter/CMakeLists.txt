add_executable(joint_states_filter joint_states_filter.cpp)
target_link_libraries(
    joint_states_filter
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES})
