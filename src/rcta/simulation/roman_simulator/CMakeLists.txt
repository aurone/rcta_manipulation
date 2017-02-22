add_executable(roman_simulator roman_simulator.cpp)
target_link_libraries(
    roman_simulator
    ${roman_client_ros_utils_LIBRARIES}
    ${roscpp_LIBRARIES})
add_dependencies(roman_simulator ${catkin_EXPORTED_TARGETS})
