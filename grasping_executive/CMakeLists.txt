cmake_minimum_required(VERSION 2.8)
project(grasping_executive)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        actionlib_msgs
        cmu_manipulation_msgs
        control_msgs
        eigen_conversions
        gascan_grasp_planning
        geometry_msgs
        grasp_planner_interface
        leatherman
        message_filters
        moveit_core
        moveit_msgs
        moveit_ros_move_group
        moveit_ros_planning
        moveit_ros_planning_interface
        pcl_conversions
        pcl_ros
        pluginlib
        rcta_manipulation_common
        robotiq_controllers
        roscpp
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

add_action_files(DIRECTORY action FILES ObjectDetection.action)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(
    costmap_object_detection
    src/costmap_object_detection/costmap_object_detection.cpp)
target_include_directories(
    costmap_object_detection
    SYSTEM PRIVATE
    ${leatherman_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${spellbook_INCLUDE_DIRS})
target_link_libraries(
    costmap_object_detection
    PRIVATE
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES})

add_executable(
    grasp_object_executor
    src/grasp_object_executor/grasp_object_executor.cpp)
target_include_directories(
    grasp_object_executor
    SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${actionlib_INCLUDE_DIRS}
    ${cmu_manipulation_msgs_INCLUDE_DIRS}
    ${control_msgs_INCLUDE_DIRS}
    ${eigen_conversions_INCLUDE_DIRS}
    ${grasp_planner_interface_INCLUDE_DIRS}
    ${leatherman_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
    ${moveit_core_INCLUDE_DIRS}
    ${moveit_ros_planning_INCLUDE_DIRS}
    ${moveit_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${rcta_manipulation_common_INCLUDE_DIRS}
    ${robotiq_controllers_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${smpl_INCLUDE_DIRS}
    ${smpl_ros_INCLUDE_DIRS}
    ${spellbook_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
    ${tf_conversions_INCLUDE_DIRS})
target_link_libraries(
    grasp_object_executor
    PRIVATE
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
target_include_directories(
    gascan_grasp_planner_plugin
    SYSTEM PRIVATE
    ${gascan_grasp_planning_INCLUDE_DIRS}
    ${grasp_planner_interface_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS})
target_link_libraries(
    gascan_grasp_planner_plugin
    PRIVATE
    ${gascan_grasp_planning_LIBRARIES}
    ${grasp_planner_interface_LIBRARIES}
    ${pluginlib_LIBRARIES})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    TARGETS
        costmap_object_detection
        gascan_grasp_planner_plugin
        grasp_object_executor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

