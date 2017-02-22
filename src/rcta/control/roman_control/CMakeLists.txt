set(COMMON_LIBS 
    ${actionlib_LIBRARIES}
    ${control_msgs_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${roman_client_ros_utils_LIBRARIES})

add_executable(
    right_limb_joint_trajectory_controller
    right_limb_joint_trajectory_controller.cpp
    roman_joint_trajectory_controller.cpp)
target_link_libraries(right_limb_joint_trajectory_controller ${COMMON_LIBS})
add_dependencies(right_limb_joint_trajectory_controller ${catkin_EXPORTED_TARGETS})

add_executable(
    left_limb_joint_trajectory_controller
    left_limb_joint_trajectory_controller.cpp
    roman_joint_trajectory_controller.cpp)
target_link_libraries(left_limb_joint_trajectory_controller ${COMMON_LIBS})
add_dependencies(left_limb_joint_trajectory_controller ${catkin_EXPORTED_TARGETS})

add_executable(roman_gripper_controller roman_gripper_controller.cpp)
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
