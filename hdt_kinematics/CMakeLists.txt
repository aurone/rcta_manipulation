cmake_minimum_required(VERSION 2.8)
project(hdt_kinematics)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        leatherman
        roscpp
        sbpl_geometry_utils
        smpl)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES hdt_robot_model hdt_kinematics
    DEPENDS Boost urdfdom_headers)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
include_directories(${urdfdom_headers_INCLUE_DIRS})
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(include)

set(
    IKFAST_COMPILE_FLAGS 
    "-DIK_FAST_HAS_LIBRARY -DIKFAST_NO_MAIN -Wno-unused-variable -Wno-unused-but-set-variable")
add_library(
    hdt_robot_model 
    src/RobotModel.cpp 
    src/kinematics.cpp 
    src/hdt_arm_transform6d.cpp)
set_target_properties(
    hdt_robot_model 
    PROPERTIES 
    COMPILE_FLAGS 
    "${IKFAST_COMPILE_FLAGS}")
target_link_libraries(
    hdt_robot_model 
    lapack 
    blas 
    ${Boost_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${urdfdom_headers_LIBRARIES})

add_library(hdt_forward_velocities src/forward_velocities.cpp)
target_link_libraries(
    hdt_forward_velocities 
    ${orocos_kdl_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${urdfdom_LIBRARIES} 
    hdt_robot_model)

install(
    DIRECTORY include/hdt_kinematics/ 
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    TARGETS hdt_robot_model hdt_forward_velocities
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

