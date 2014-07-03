cmake_minimum_required(VERSION 2.8)
project(hdt)

### EXTERNAL DEPENDENCIES ###

# Boost
find_package(Boost REQUIRED system thread)

# catkin
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs 
    eigen_conversions
    geometric_shapes
    geometry_msgs
    interactive_markers
    industrial_msgs
    industrial_robot_client
    leatherman 
    message_generation 
    moveit_ros_planning 
    pluginlib
    pr2_vfh_database
    roscpp 
    rviz 
    sbpl_geometry_utils 
    sbpl_manipulation
    sensor_msgs 
    simple_message
    std_msgs 
    tf 
    trajectory_msgs
    urdf 
    urdf_parser
    visualization_msgs)

# Eigen
find_package(Eigen REQUIRED)

# PkgConfig
find_package(PkgConfig REQUIRED)

# lua5.2
pkg_check_modules(lua REQUIRED lua5.2)

# Openni
pkg_check_modules(openni REQUIRED libopenni)
include_directories(${openni_INCLUDE_DIRS})
link_directories(${openni_LIBRARY_DIRS})

# Qt4
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# required to locate compiled headers for designer forms
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 

add_message_files(FILES
    DiagnosticStatus.msg 
    JointDiagnosticStatus.msg 
    EmergencyStop.msg 
    ClearEmergencyStop.msg
    AcknowledgeReset.msg
    TeleopDiagnosticStatus.msg)
add_service_files(FILES MoveArmCommand.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

set(CMAKE_INCLUDE_CURRENT_DIR On)
set(CMAKE_VERBOSE_MAKEFILE Off)
set(CMAKE_COLOR_MAKEFILE On)

set(BUILD_SHARED_LIBS On)
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(src)
add_subdirectory(config)

