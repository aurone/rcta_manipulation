add_library(hdt_arm_planning HDTRobotModel.cpp)
target_link_libraries(
    hdt_arm_planning
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_arm_planner_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_robot_model)

add_executable(hdt_manipulation_test hdt_manipulation_test.cpp)
target_link_libraries(
    hdt_manipulation_test
    hdt_arm_planning
    ${moveit_core_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_arm_planner_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${sbpl_kdl_robot_model_LIBRARIES})

add_executable(move_arm_node move_arm_node.cpp MoveArmNode.cpp)
target_link_libraries(
    move_arm_node
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_arm_planner_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${sbpl_kdl_robot_model_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_model_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_arm_planning
    hdt_robot_model)

install(
    TARGETS hdt_arm_planning move_arm_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
