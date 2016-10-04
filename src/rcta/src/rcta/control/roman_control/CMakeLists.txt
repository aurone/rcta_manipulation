add_executable(
    roman_joint_trajectory_controller
    roman_joint_trajectory_controller.cpp)
target_link_libraries(
    roman_joint_trajectory_controller
    ${actionlib_LIBRARIES}
    ${control_msgs_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roman_client_ros_utils_LIBRARIES})
add_dependencies(roman_joint_trajectory_controller ${catkin_EXPORTED_TARGETS})

add_executable(
    roman_gripper_controller
    roman_gripper_controller.cpp)
target_link_libraries(
    roman_gripper_controller
    ${actionlib_LIBRARIES}
    ${control_msgs_LBIRARIES}
    ${roscpp_LIBRARIES}
    ${roman_client_ros_utils_LIBRARIES}
    robotiq_common)
add_dependencies(
    roman_gripper_controller
    ${catkin_EXPORTED_TARGETS})
