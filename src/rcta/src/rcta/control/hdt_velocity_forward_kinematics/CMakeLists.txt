add_executable(forward_velocities forward_velocities.cpp)
target_link_libraries(
    forward_velocities
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_robot_model)

