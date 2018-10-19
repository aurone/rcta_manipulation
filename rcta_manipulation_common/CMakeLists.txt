cmake_minimum_required(VERSION 2.8.12)
project(rcta_manipulation_common)

find_package(catkin REQUIRED COMPONENTS geometry_msgs moveit_msgs octomap_msgs sensor_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs moveit_msgs octomap_msgs sensor_msgs)

install(
    DIRECTORY include/rcta_manipulation_common/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
