add_executable(reposition_base_simulator reposition_base_simulator.cpp)
target_link_libraries(
    reposition_base_simulator
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES})
add_dependencies(reposition_base_simulator ${catkin_EXPORTED_TARGETS})

