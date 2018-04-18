cmake_minimum_required(VERSION 2.8.12)
project(grasp_generator_plugin)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        pluginlib
        grasp_generator_msgs
        grasp_planner_interface)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

add_library(grasp_generator_plugin src/grasp_generator_plugin.cpp)
target_link_libraries(
    grasp_generator_plugin
    ${actionlib_LIBRARIES}
    ${grasp_planner_interface_LIBRARIES})

install(
    TARGETS grasp_generator_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


