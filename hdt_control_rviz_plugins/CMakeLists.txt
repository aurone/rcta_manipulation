cmake_minimum_required(VERSION 2.8.12)
project(hdt_control_rviz_plugins)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Qt4 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        eigen_conversions
        hdt_control_msgs
        hdt_kinematics
        rcta_manipulation_common
        roscpp
        spellbook
        tf
        rviz)

find_package(smpl REQUIRED)

include(${QT_USE_FILE})

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})

#################################
# Build Controller Status Panel #
#################################

qt4_wrap_cpp(MOC_HEADER_SOURCES src/controller_status_panel_widget.h)
qt4_wrap_ui(MOC_FORM_HEADERS src/controller_status_panel_widget.ui)
add_library(
    hdt_controller_plugins
    ${MOC_HEADER_SOURCES}
    ${MOC_FORM_HEADERS}
    src/controller_status_panel_widget.cpp)
target_link_libraries(
    hdt_controller_plugins
    ${QT_LIBRARIES}
    ${smpl_LIBRARIES}
    qwt)

list(APPEND BUILT_LIBRARIES hdt_controller_plugins)

##################################
# Build Temperature Status Panel #
##################################

if(0)
    qt4_wrap_ui(MOC_FORM_HEADERS src/TemperatureStatusPanelWidget.ui)
    qt4_wrap_cpp(MOC_HEADER_SOURCES src/TemperatureStatusPanel.h)

    add_library(
        temperature_status_panel
        ${MOC_HEADER_SOURCES}
        ${MOC_FORM_HEADERS}
        src/TemperatureStatusPanel.cpp)
    target_link_libraries(
        temperature_status_panel
        ${QT_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${rviz_LIBRARIES}
        qwt)

    list(APPEND BUILT_LIBRARIES temperature_status_panel)
endif()

#############################
# Build Teleop Status Panel #
#############################

qt4_wrap_cpp(TELEOP_PLUGIN_MOC_HEADER_SOURCES src/teleop_status_panel.h)
qt4_wrap_ui(TELEOP_PLUGIN_MOC_FORM_HEADERS src/teleop_status_panel.ui)

add_library(
    hdt_arm_teleop_plugin
    ${TELEOP_PLUGIN_MOC_HEADER_SOURCES}
    ${TELEOP_PLUGIN_MOC_FORM_HEADERS}
    src/teleop_status_panel.cpp)
target_link_libraries(
    hdt_arm_teleop_plugin
    ${QT_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES})

list(APPEND BUILT_LIBRARIES hdt_arm_teleop_plugin)

###############################
# Build Viservo Command Panel #
###############################

qt4_wrap_cpp(VISERVO_COMMAND_PANEL_MOC_HEADER_SOURCES src/viservo_command_panel.h)
add_library(
    viservo_command_panel_plugin
    ${VISERVO_COMMAND_PANEL_MOC_HEADER_SOURCES}
    src/viservo_command_panel.cpp)
target_link_libraries(
    viservo_command_panel_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${rviz_LIBRARIES}
    ${tf_LIBRARIES})

list(APPEND BUILT_LIBRARIES viservo_command_panel_plugin)

install(
    TARGETS ${BUILT_LIBRARIES}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
