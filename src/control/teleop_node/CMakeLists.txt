add_executable(hdt_arm_teleop hdt_arm_teleop.cpp teleop_node.cpp controller.cpp)
target_link_libraries(hdt_arm_teleop ${catkin_LIBRARIES} config_block InputDevice)

install(TARGETS hdt_arm_teleop RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

