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
        cmu_manipulation_msgs
        eigen_conversions
        gascan_grasp_planning
        geometric_shapes
        geometry_msgs
        grasp_planner_interface
        interactive_markers
        leatherman
        moveit_core
        moveit_msgs
        moveit_ros_move_group
        moveit_ros_planning
        moveit_ros_planning_interface
        octomap_msgs
        pcl_conversions
        pcl_ros
        pluginlib
        rcta_manipulation_common
        robotiq_controllers
        roscpp
        rviz
        sensor_msgs
        smpl_ros
        spellbook
        tf
        tf_conversions
        trajectory_msgs)

find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(sbpl REQUIRED)
find_package(smpl REQUIRED)

add_action_files(DIRECTORY action FILES MoveArm.action ObjectDetection.action)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        octomap_msgs
        moveit_msgs
        trajectory_msgs)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include(${QT_USE_FILE})

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${urdfdom_headers_INCLUDE_DIRS})

add_executable(
    costmap_object_detection
    src/costmap_object_detection/costmap_object_detection.cpp)
target_link_libraries(
    costmap_object_detection
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES})

add_executable(
    grasp_object_executor
    src/grasp_object_executor/grasp_object_executor.cpp)
target_link_libraries(
    grasp_object_executor
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${grasp_planner_interface_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    ${pcl_ros_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${tf_LIBRARIES}
    ${robotiq_controllers_LIBRARIES})
add_dependencies(grasp_object_executor ${PROJECT_NAME}_generate_messages_cpp)

add_library(
    gascan_grasp_planner_plugin
    src/grasp_object_executor/gascan_grasp_planner_plugin.cpp)
target_link_libraries(
    gascan_grasp_planner_plugin
    ${gascan_grasp_planning_LIBRARIES}
    ${grasp_planner_interface_LIBRARIES}
    ${pluginlib_LIBRARIES})

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
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${spellbook_LIBRARIES}
    ${tf_LIBRARIES}
    ${robotiq_controllers_LIBRARIES}
    qwt)

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
        costmap_object_detection
        grasp_object_executor
        grasping_command_panel_plugin
        gripper_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

