add_executable(static_world_publisher WorldSimulatorNode.cpp world_simulator.cpp)
target_link_libraries(
    static_world_publisher
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES})
