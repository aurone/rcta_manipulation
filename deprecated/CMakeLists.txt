cmake_minimum_required(VERSION 2.8)
project(rcta)

### EXTERNAL DEPENDENCIES ###

find_package(Boost REQUIRED system thread)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

find_package(catkin REQUIRED COMPONENTS
    ar_track_alvar
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    eigen_conversions
    geometric_shapes
    geometry_msgs
    #genmsg # The actionlib tutorial says this is required for building messages, but everything seems to be OK?
    cmu_manipulation_msgs
    interactive_markers
    leatherman
    message_generation
    moveit_core
    moveit_planners_sbpl
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    nav_msgs
    octomap_msgs
    pluginlib
    roman_client_ros_utils
    rosbag
    roscpp
    rospack
    rviz
    smpl
    sbpl_collision_checking
    sbpl_geometry_utils
    spellbook_ros
    sensor_msgs
    std_msgs
    tf
    trajectory_msgs
    urdf
    visualization_msgs)

# has to come after cmake_modules component is found by find_package(catkin ...
find_package(Eigen REQUIRED)

pkg_check_modules(glew REQUIRED glew)
pkg_check_modules(lua REQUIRED lua5.2)
pkg_check_modules(ncurses REQUIRED ncurses)
pkg_check_modules(openni REQUIRED libopenni)
find_package(sbpl REQUIRED)
pkg_check_modules(sdl REQUIRED sdl2)
find_package(urdfdom REQUIRED)

# project settings
set(CMAKE_INCLUDE_CURRENT_DIR On)
set(CMAKE_VERBOSE_MAKEFILE Off)
set(CMAKE_COLOR_MAKEFILE On)

set(BUILD_SHARED_LIBS On)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)

include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

include_directories(${openni_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${glew_INCLUDE_DIRS})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
# required to locate compiled headers for designer forms
include_directories(${CMAKE_CURRENT_BINARY_DIR})

link_directories(${openni_LIBRARY_DIRS})
link_directories(${glew_LIBRARY_DIRS})

add_action_files(
    DIRECTORY action
    FILES
        ObjectDetection.action
        MoveArm.action
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

catkin_package(
    INCLUDE_DIRS
		src/rcta/common
	CATKIN_DEPENDS
        message_runtime
	LIBRARIES
		hdt_viz)

add_subdirectory(src)
add_subdirectory(config)
