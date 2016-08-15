add_executable(grasp_object_executor grasp_object_executor.cpp GraspObjectExecutor.cpp)
target_link_libraries(
    grasp_object_executor
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model
    robotiq_common
    costmap_extruder)
add_dependencies(grasp_object_executor ${PROJECT_NAME}_generate_messages_cpp)

