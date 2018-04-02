cmake_minimum_required(VERSION 2.8.12)
project(roman_moveit_plugins)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        moveit_core
        roman_client_ros_utils
        pluginlib)

catkin_package()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

add_library(add_jpl_time_parameterization src/add_jpl_time_parameterization.cpp)
target_link_libraries(
    add_jpl_time_parameterization
    ${moveit_core_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${roman_client_ros_utils_LIBRARIES})

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS add_jpl_time_parameterization
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

