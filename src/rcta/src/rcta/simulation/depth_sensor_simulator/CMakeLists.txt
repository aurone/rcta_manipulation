add_executable(depth_sensor_simulator depth_sensor_simulator.cpp DepthSensorSimulatorNode.cpp)
target_link_libraries(
        depth_sensor_simulator
        ${rospack_LIBRARIES}
        ${sbpl_geometry_utils_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${sdl_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${glew_LIBRARIES})

