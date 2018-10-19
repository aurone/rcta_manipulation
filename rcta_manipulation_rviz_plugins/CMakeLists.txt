cmake_minimum_required(VERSION 2.8)
project(rcta_manipulation_rviz_plugins)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Stealing this option name from smpl_moveit_interface so a single variable can
# be passed on the catkin CLI.
option(SMPL_MOVEIT_INTERFACE_QT5 "Build RViz plugins against Qt5" OFF)

if(SMPL_MOVEIT_INTERFACE_QT5)
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Core REQUIRED)
else()
    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        cmu_manipulation_msgs
        control_msgs
        eigen_conversions
        geometric_shapes
        geometry_msgs
        interactive_markers
        moveit_core
        moveit_ros_planning
        rcta_manipulation_common
        robotiq_controllers
        roscpp
        rviz
        sensor_msgs
        spellbook
        std_msgs
        tf
        tf_conversions
        visualization_msgs)
find_package(smpl REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

if(SMPL_MOVEIT_INTERFACE_QT5)
    set(MY_QT_LIBRARIES Qt5::Core Qt5::Widgets)
else()
    set(MY_QT_LIBRARIES ${QT_LIBRARIES})
endif()

if(SMPL_MOVEIT_INTERFACE_QT5)
    qt5_wrap_cpp(
        GRASPING_COMMAND_PANEL_MOC_HEADER_SOURCES
        src/grasping_command_panel/grasping_command_panel.h)
else()
    qt4_wrap_cpp(
        GRASPING_COMMAND_PANEL_MOC_HEADER_SOURCES
        src/grasping_command_panel/grasping_command_panel.h)
endif()
add_library(
    grasping_command_panel_plugin
    ${GRASPING_COMMAND_PANEL_MOC_HEADER_SOURCES}
    src/grasping_command_panel/grasping_command_panel.cpp)
target_include_directories(
    grasping_command_panel_plugin
    SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})
target_link_libraries(
    grasping_command_panel_plugin
    PRIVATE
    ${MY_QT_LIBRARIES}
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

if(SMPL_MOVEIT_INTERFACE_QT5)
    qt5_wrap_cpp(
        RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES
        src/gripper_command_panel/gripper_command_panel.h)
else()
    qt4_wrap_cpp(
        RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES
        src/gripper_command_panel/gripper_command_panel.h)
endif()
add_library(
    gripper_command_panel_plugin
    ${RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES}
    src/gripper_command_panel/gripper_command_panel.cpp)
target_include_directories(
    gripper_command_panel_plugin
    SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})
target_link_libraries(
    gripper_command_panel_plugin
    PRIVATE
    ${MY_QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${robotiq_controllers_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES})

install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    TARGETS
        grasping_command_panel_plugin
        gripper_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
