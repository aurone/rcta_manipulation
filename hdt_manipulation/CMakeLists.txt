cmake_minimum_required(VERSION 2.8.12)
project(hdt_manipulation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        leatherman
        roscpp
        smpl
        spellbook_ros
        tf
        hdt_kinematics)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_library(hdt_arm_planning src/HDTRobotModel.cpp)
target_link_libraries(
    hdt_arm_planning
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS hdt_arm_planning
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

