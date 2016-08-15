add_executable(joint_position_offset_test joint_position_offset_test.cpp)
target_link_libraries(
    joint_position_offset_test
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    hdt_robot_model)

