add_executable(stow_arm stow_arm.cpp)
target_link_libraries(
    stow_arm
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    hdt_robot_model
    msg_utils)

