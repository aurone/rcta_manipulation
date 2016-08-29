add_executable(
    hdt_base_planning_node
    hdt_base_planning.cpp
    RepositionBaseExecutor.cpp
    xytheta_collision_checker.cpp)

target_link_libraries(
    hdt_base_planning_node
    ${actionlib_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_model_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${SBPL_LIBRARIES}
    msg_utils
    hdt_robot_model
    robotiq_common)

add_dependencies(
    hdt_base_planning_node
    ${PROJECT_NAME}_generate_messages_cpp)
