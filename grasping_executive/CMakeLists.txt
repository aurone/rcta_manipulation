cmake_minimum_required(VERSION 2.8)
project(grasping_executive)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Qt4 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        actionlib_msgs 
        eigen_conversions
        gascan_grasp_planning
        geometric_shapes
        geometry_msgs 
        hdt_control_msgs
        hdt_kinematics
        interactive_markers
        leatherman
        moveit_core
        moveit_msgs 
        moveit_ros_move_group
        moveit_ros_planning
        moveit_ros_planning_interface
        octomap_msgs 
        pcl_ros
        pluginlib
        rcta_manipulation_common
        robotiq_controllers
        roscpp
        rviz
        sensor_msgs 
        smpl
        smpl_ros
        spellbook_ros
        tf
        tf_conversions
        trajectory_msgs)

find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(sbpl REQUIRED)

add_action_files(DIRECTORY action FILES MoveArm.action ObjectDetection.action)

generate_messages(
    DEPENDENCIES
        actionlib_msgs 
        geometry_msgs 
        sensor_msgs 
        octomap_msgs 
        moveit_msgs 
        trajectory_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs octomap_msgs moveit_msgs trajectory_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include(${QT_USE_FILE})

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${urdfdom_headers_INCLUDE_DIRS})

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
    ${hdt_kinematics_LIBRARIES})

add_executable(move_arm_node src/move_arm_node/move_arm_node.cpp)
target_link_libraries(
    move_arm_node 
    hdt_arm_planning 
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
    ${hdt_kinematics_LIBRARIES})

add_executable(
    costmap_object_detection 
    src/costmap_object_detection/costmap_object_detection.cpp)
target_link_libraries(
    costmap_object_detection 
    ${leatherman_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${spellbook_ros_LIBRARIES})

add_executable(
    retrieve_object_simulator 
    src/retrieve_object_simulator/CollisionModel2.cpp 
    src/retrieve_object_simulator/RetrieveObjectSimulator.cpp 
    src/retrieve_object_simulator/retrieve_object_simulator.cpp)
target_link_libraries(
    retrieve_object_simulator 
    ${SBPL_LIBRARIES} 
    ${actionlib_LIBRARIES} 
    ${eigen_conversions_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${spellbook_ros_LIBRARIES} 
    ${hdt_kinematics_LIBRARIES})

add_executable(stow_arm src/stow_arm/stow_arm.cpp)
target_link_libraries(
    stow_arm 
    ${actionlib_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${hdt_kinematics_LIBRARIES} 
    ${spellbook_ros_LIBRARIES})
add_dependencies(stow_arm ${PROJECT_NAME}_generate_messages_cpp)

add_executable(
    grasp_object_executor 
    src/grasp_object_executor/grasp_object_executor.cpp)
target_link_libraries(
    grasp_object_executor
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${tf_LIBRARIES}
    ${gascan_grasp_planning_LIBRARIES}
    ${hdt_kinematics_LIBRARIES}
    ${robotiq_controllers_LIBRARIES})
add_dependencies(grasp_object_executor ${PROJECT_NAME}_generate_messages_cpp)

qt4_wrap_cpp(
    HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES 
    src/grasping_command_panel/grasping_command_panel.h)
add_library(
    grasping_command_panel_plugin 
    ${HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES} 
    src/grasping_command_panel/grasping_command_panel.cpp)
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
    ${hdt_kinematics_LIBRARIES}
    ${robotiq_controllers_LIBRARIES}
    qwt)

#qt4_wrap_cpp(MOC_HEADER_SOURCES src/pick_and_place_panel/PickAndPlacePanel.h)
#add_library(
#    hdt_pick_and_place_plugin
#    ${MOC_HEADER_SOURCES}
#    src/pick_and_place_panel/PickAndPlacePanel.cpp
#    src/pick_and_place_panel/SimpleInteractiveMarker.cpp
#    src/pick_and_place_panel/SimpleInteractiveMarkerServer.cpp
#    src/pick_and_place_panel/GraspMarkerSelectionMarker.cpp)
#target_link_libraries(
#    hdt_pick_and_place_plugin
#    ${QT_LIBRARIES}
#    ${actionlib_LIBRARIES}
#    ${eigen_conversions_LIBRARIES}
#    ${interactive_markers_LIBRARIES}
#    ${pcl_ros_LIBRARIES}
#    ${pr2_vfh_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${rviz_LIBRARIES}
#    ${tf_LIBRARIES}
#    ${robotiq_controllers_LIBRARIES})

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

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    TARGETS
        hdt_arm_planning 
        move_arm_node 
        costmap_object_detection
        retrieve_object_simulator
        stow_arm
        grasp_object_executor
        grasping_command_panel_plugin 
#        hdt_pick_and_place_plugin
        gripper_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

