cmake_minimum_required(VERSION 2.8.12)
project(hdt_control)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# TODO: qwt
# TODO: ncurses
# TODO: config_block
# TODO: orocos_kdl for viservo_controller
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(lua52 REQUIRED lua5.2)
pkg_check_modules(ncurses REQUIRED ncurses)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        ar_track_alvar_msgs
        actionlib
        eigen_conversions
        hdt_control_msgs
        roscpp
        tf
        rcta_manipulation_common
        rviz
        spellbook
        sbpl_kdl_robot_model
        sbpl_geometry_utils
        hdt_kinematics)
find_package(orocos_kdl REQUIRED)
find_package(smpl REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${lua52_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_definitions(-DROS=1)

link_directories("/usr/local/esd/lib64")

set(
    hdt_LIBRARIES
    HDT
    InputDevice
    ManipulatorError
    ManipulatorInterface
    ManipulatorParameters
    ntcan)

add_executable(
    attached_markers_filter
    src/attached_markers_filter/attached_markers_filter.cpp)
target_link_libraries(
    attached_markers_filter
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${spellbook_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

add_executable(
    measure_eef_error
    src/error_measurement_node/measure_eef_error.cpp)
target_link_libraries(
    measure_eef_error
    ${eigen_conversions_LIBRARIES}
    ${hdt_kinematics_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES}
    ${tf_LIBRARIES})

add_executable(
    fake_marker_pub
    src/fake_marker_publisher/fake_marker_pub.cpp)
target_link_libraries(
    fake_marker_pub
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${smpl_LIBRARIES})

#add_executable(
#    motion_download_interface_HDT
#    src/hdt_arm_driver/HDTManipulator.cpp
#    src/hdt_arm_driver/motion_download_interface.cpp)
#target_link_libraries(
#    motion_download_interface_HDT
#    ${ncurses_LIBRARIES}
#    ${hdt_LIBRARIES}
#    pthread
#    ${roscpp_LIBRARIES})

add_executable(
    joint_trajectory_action_HDT
    src/hdt_arm_driver/joint_trajectory_action.cpp)
target_link_libraries(
    joint_trajectory_action_HDT
    ${Boost_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

#add_executable(
#    hdt_arm_driver
#    src/hdt_arm_driver/hdt_arm_driver.cpp
#    src/hdt_arm_driver/manipulator_interface_ros.cpp
#    src/hdt_arm_driver/manipulator_interface_live_ros.cpp
#    src/hdt_arm_driver/manipulator_interface_sim_ros.cpp)
#target_link_libraries(
#    hdt_arm_driver
#    ${actionlib_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${sbpl_geometry_utils_LIBRARIES}
#    ${lua52_LIBRARIES}
#    ${hdt_LIBRARIES}
#    config_block
#    ${hdt_kinematics_LIBRARIES})

add_executable(
    joint_position_offset_test
    src/joint_position_offset_test/joint_position_offset_test.cpp)
target_link_libraries(
    joint_position_offset_test
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

add_executable(
    joint_states_filter
    src/joint_states_filter/joint_states_filter.cpp)
target_link_libraries(
    joint_states_filter
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES})

#add_executable(
#    repeatability_measure
#    src/repeatability/repeatability_measure.cpp)
#target_link_libraries(
#    repeatability_measure
#    ${actionlib_LIBRARIES}
#    ${eigen_conversions_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${spellbook_LIBRARIES}
#    ${tf_LIBRARIES}
#    ${hdt_kinematics_LIBRARIES})

#add_executable(
#    hdt_arm_teleop
#    src/teleop_node/hdt_arm_teleop.cpp
#    src/teleop_node/teleop_node.cpp
#    src/teleop_node/controller.cpp)
#target_link_libraries(
#    hdt_arm_teleop
#    ${roscpp_LIBRARIES}
#    ${urdf_LIBRARIES}
#    ${urdf_parser_LIBRARIES}
#    config_block
#    InputDevice)

add_executable(
    trajectory_follower
    src/trajectory_follower/trajectory_follower.cpp)
target_link_libraries(
    trajectory_follower
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

#add_executable(
#    viservo_control_executor
#    src/viservo_controller/ViservoControlExecutor.cpp)
#target_link_libraries(
#    viservo_control_executor
#    ${actionlib_LIBRARIES}
#    ${eigen_conversions_LIBRARIES}
#    ${orocos_kdl_LIBRARIES}
#    ${sbpl_geometry_utils_LIBRARIES}
#    ${sbpl_kdl_robot_model_LIBRARIES}
#    ${spellbook_LIBRARIES}
#    ${tf_LIBRARIES}
#    ${urdf_parser_LIBRARIES}
#    ${hdt_kinematics_LIBRARIES})

install(
    TARGETS
        measure_eef_error
        fake_marker_pub
#        motion_download_interface_HDT
        joint_trajectory_action_HDT
#        hdt_arm_driver
#        temperature_status_panel
        joint_position_offset_test
        joint_states_filter
#        repeatability_measure
#        hdt_arm_teleop
        trajectory_follower
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
