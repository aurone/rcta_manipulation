cmake_minimum_required(VERSION 2.8.12)
project(grasp_planner_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        eigen_stl_containers
        roscpp
        spellbook
        visualization_msgs)

catkin_package(
    LIBRARIES grasp_planner_interface
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS eigen_stl_containers roscpp visualization_msgs
    DEPENDS PCL)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

add_library(
    grasp_planner_interface
    src/grasp_planner_plugin.cpp
    src/grasp_utils.cpp)
target_link_libraries(
    grasp_planner_interface
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES})

install(
    DIRECTORY include/grasp_planner_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    TARGETS grasp_planner_interface
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

