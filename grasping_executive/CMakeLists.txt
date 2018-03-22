cmake_minimum_required(VERSION 2.8)
project(grasping_executive)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib 
        eigen_conversions 
        leatherman 
        moveit_core 
        moveit_ros_planning
        robotiq_controllers
        roscpp
        smpl
        spellbook_ros
        tf_conversions
        tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(hdt_arm_planning src/HDTRobotModel.cpp)
target_link_libraries(
    hdt_arm_planning
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_LIBRARIES}
    ${tf_LIBRARIES}
    ${urdf_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_robot_model)

add_executable(move_arm_node src/move_arm_node.cpp src/MoveArmNode.cpp)
target_link_libraries(
    move_arm_node 
    ${actionlib_LIBRARIES} 
    ${eigen_conversions_LIBRARIES} 
    ${leatherman_LIBRARIES} 
    ${moveit_core_LIBRARIES} 
    ${moveit_ros_move_group_LIBRARIES} 
    ${moveit_ros_planning_LIBRARIES} 
    ${moveit_ros_planning_interface_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${smpl_LIBRARIES} 
    ${sbpl_collision_checking_LIBRARIES} 
    ${sbpl_kdl_robot_model_LIBRARIES} 
    ${spellbook_ros_LIBRARIES} 
    ${tf_LIBRARIES} 
    ${urdf_model_LIBRARIES} 
    ${urdf_parser_LIBRARIES} 
    hdt_arm_planning 
    hdt_robot_model)

add_executable(costmap_object_detection src/costmap_object_detection.cpp)
target_link_libraries(
    costmap_object_detection 
    ${leatherman_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    msg_utils)

add_executable(
    retrieve_object_simulator 
    src/CollisionModel2.cpp 
    src/RetrieveObjectSimulator.cpp 
    src/retrieve_object_simulator.cpp)
target_link_libraries(
    retrieve_object_simulator 
    ${SBPL_LIBRARIES} 
    ${actionlib_LIBRARIES} 
    ${eigen_conversions_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${spellbook_ros_LIBRARIES} 
    hdt_robot_model)

add_executable(stow_arm src/stow_arm.cpp)
target_link_libraries(
    stow_arm 
    ${actionlib_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    hdt_robot_model 
    msg_utils)

add_executable(
    grasp_object_executor 
    src/grasp_object_executor.cpp 
    src/GraspObjectExecutor.cpp)
target_link_libraries(
    grasp_object_executor
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${tf_LIBRARIES}
    gascan_grasp_planner
    hdt_robot_model
    robotiq_common
    costmap_extruder)
add_dependencies(grasp_object_executor ${PROJECT_NAME}_generate_messages_cpp)

qt4_wrap_cpp(HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES src/grasping_command_panel.h)
add_library(
    grasping_command_panel_plugin 
    ${HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES} 
    grasping_command_panel.cpp)
target_link_libraries(
    grasping_command_panel_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model
    robotiq_common
    qwt)

qt4_wrap_cpp(MOC_HEADER_SOURCES src/PickAndPlacePanel.h)
add_library(
    hdt_pick_and_place_plugin
    ${MOC_HEADER_SOURCES}
    PickAndPlacePanel.cpp
    SimpleInteractiveMarker.cpp
    SimpleInteractiveMarkerServer.cpp
    GraspMarkerSelectionMarker.cpp)
target_link_libraries(
    hdt_pick_and_place_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${pcl_ros_LIBRARIES}
    ${pr2_vfh_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${tf_LIBRARIES}
    robotiq_common)

qt4_wrap_cpp(
    RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES 
    src/gripper_command_panel/gripper_command_panel.h)
add_library(
    gripper_command_panel_plugin 
    ${RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES}
    src/gripper_command_panel/gripper_command_panel.cpp)
target_link_libraries(
    gripper_command_panel_plugin 
    ${QT_LIBRARIES} 
    ${actionlib_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${robotiq_controllers_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES})

install(
    TARGETS
        hdt_arm_planning 
        move_arm_node 
        grasping_command_panel_plugin 
        hdt_pick_and_place_plugin
        gripper_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

