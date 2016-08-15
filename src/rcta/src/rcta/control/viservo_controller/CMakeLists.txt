add_executable(viservo_control_executor ViservoControlExecutor.cpp)
target_link_libraries(
    viservo_control_executor
    ${actionlib_LIBRARIES}
    ${catkin_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${sbpl_kdl_robot_model_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_robot_model)

