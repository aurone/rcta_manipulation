cmake_minimum_required(VERSION 2.8.12)
project(roman_moveit_plugins)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        message_filters
        moveit_core
        octomap_msgs
        pluginlib
        roscpp
        tf)

catkin_package()

add_library(octomap_occupancy_map_updater src/octomap_occupancy_map_updater.cpp)
target_compile_options(octomap_occupancy_map_updater PRIVATE -std=c++11)
target_include_directories(octomap_occupancy_map_updater SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(
    octomap_occupancy_map_updater
    ${moveit_core_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${message_filters_LIBRARIES}
    ${octomap_msgs_LIBRARIES}
    ${tf_LIBRARIES})

install(
    TARGETS octomap_occupancy_map_updater
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    FILES octomap_occupancy_map_updater_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (0)
find_package(catkin REQUIRED COMPONENTS roman_client_ros_utils)

add_library(add_jpl_time_parameterization src/add_jpl_time_parameterization.cpp)
target_compile_options(add_jpl_time_parameterization PRIVATE -std=c++11)
target_include_directories(add_jpl_time_parameterization SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(
    add_jpl_time_parameterization
    ${moveit_core_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${roman_client_ros_utils_LIBRARIES})

install(
    TARGETS add_jpl_time_parameterization
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
