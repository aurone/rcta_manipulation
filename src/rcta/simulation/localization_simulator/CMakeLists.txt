add_executable(localization_simulator LocalizationSimulator.cpp localization_simulator.cpp)
target_link_libraries(
    localization_simulator
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${rosbag_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES})
add_dependencies(localization_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS})
