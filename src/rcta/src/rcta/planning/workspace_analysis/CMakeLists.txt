add_executable(analyze_workspace analyze_workspace.cpp)
target_link_libraries(analyze_workspace ${moveit_core_LIBRARIES} ${moveit_ros_planning_LIBRARIES})
