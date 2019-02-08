cmake_minimum_required(VERSION 2.8.12)
project(kdl_kinematics_plugin)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
    roscpp
    kdl_parser
    srdfdom
    tf_conversions
    moveit_ros_planning
    random_numbers
    geometry_msgs
    moveit_msgs
    moveit_core)

find_package(urdfdom REQUIRED)

find_package(orocos_kdl REQUIRED)

catkin_package()

add_library(
    kdl_kinematics_plugin
    src/kdl_kinematics_plugin.cpp
    src/chainiksolver_pos_nr_jl_mimic.cpp
    src/chainiksolver_vel_pinv_mimic.cpp)

target_compile_options(kdl_kinematics_plugin PUBLIC -std=c++11)

target_include_directories(
    kdl_kinematics_plugin
    SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(
    kdl_kinematics_plugin
    PRIVATE
    ${catkin_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${orocos_kdl_LIBRARIES})

install(
    FILES kdl_kinematics_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS kdl_kinematics_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
