add_executable(trajectory_follower trajectory_follower.cpp TrajectoryFollower.cpp)
target_link_libraries(
    trajectory_follower
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    hdt_robot_model)

