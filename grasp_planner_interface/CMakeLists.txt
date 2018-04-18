cmake_minimum_required(VERSION 2.8.12)
project(grasp_planner_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
    LIBRARIES grasp_planner_plugin
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    CATKIN_DEPENDS roscpp sensor_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(grasp_planner_plugin src/grasp_planner_plugin.cpp)
target_link_libraries(grasp_planner_plugin ${roscpp_LIBRARIES})

install(
    DIRECTORY include/grasp_planner_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    TARGETS grasp_planner_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

