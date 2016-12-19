add_executable(
    reposition_planner_node
    hdt_base_planning.cpp
    RepositionBaseExecutor.cpp
    xytheta_collision_checker.cpp)

target_link_libraries(
    reposition_planner_node
    ${actionlib_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_model_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${smpl_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${SBPL_LIBRARIES}
    gascan_grasp_planner
    msg_utils
    hdt_robot_model
    robotiq_common)

add_dependencies(
    reposition_planner_node
    ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
