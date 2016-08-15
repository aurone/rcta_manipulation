add_executable(hdt_arm_teleop hdt_arm_teleop.cpp teleop_node.cpp controller.cpp)
target_link_libraries(
    hdt_arm_teleop
    ${roscpp_LIBRARIES}
    ${urdf_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    config_block
    InputDevice)
add_dependencies(hdt_arm_teleop ${PROJECT_NAME}_generate_messages_cpp)

install(
    TARGETS hdt_arm_teleop
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

