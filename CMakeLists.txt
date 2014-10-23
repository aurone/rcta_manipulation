cmake_minimum_required(VERSION 2.8)
project(hdt)

### EXTERNAL DEPENDENCIES ###

# Boost
find_package(Boost REQUIRED system thread)

# catkin
find_package(catkin REQUIRED COMPONENTS
    ar_track_alvar
    actionlib
    actionlib_msgs
    control_msgs 
    eigen_conversions
    geometric_shapes
    geometry_msgs
    #genmsg # The actionlib tutorial says this is required for building messages, but everything seems to be OK?
    hdt_msgs
    interactive_markers
    leatherman 
    message_generation 
    moveit_ros_planning 
    nav_msgs 
    octomap_msgs 
    pluginlib
    pr2_vfh_database
    pviz
    rosbag
    roscpp 
    rospack
    rviz 
    sbpl_geometry_utils 
    sbpl_manipulation
    sensor_msgs 
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

# sdl2
pkg_check_modules(sdl REQUIRED sdl2)

# Openni
pkg_check_modules(openni REQUIRED libopenni)
include_directories(${openni_INCLUDE_DIRS})
link_directories(${openni_LIBRARY_DIRS})

# Qt4
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

# glew
pkg_check_modules(glew REQUIRED glew)
include_directories(${glew_INCLUDE_DIRS})
link_directories(${glew_LIBRARY_DIRS})

# SBPL
find_package(sbpl REQUIRED)
include_directories(${SBPL_INCLUDE_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

# required to locate compiled headers for designer forms
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 

add_action_files(
    DIRECTORY action
    FILES
        ObjectDetection.action
        MoveArmCommand.action
        TeleportAndaliteCommand.action
        TeleportHDTCommand.action
        ViservoCommand.action)
add_message_files(
    FILES
        ControllerDiagnosticStatus.msg 
        JointDiagnosticStatus.msg 
        JointState.msg
        EmergencyStop.msg 
        ClearEmergencyStop.msg
        AcknowledgeReset.msg
        TeleopDiagnosticStatus.msg)
generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        moveit_msgs
        nav_msgs
        octomap_msgs
        sensor_msgs
        std_msgs
        trajectory_msgs)

catkin_package(INCLUDE_DIRS 
		src/hdt/common
	CATKIN_DEPENDS message_runtime
	LIBRARIES 
		hdt_viz)

set(CMAKE_INCLUDE_CURRENT_DIR On)
set(CMAKE_VERBOSE_MAKEFILE Off)
set(CMAKE_COLOR_MAKEFILE On)

set(BUILD_SHARED_LIBS On)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(src)
add_subdirectory(config)

