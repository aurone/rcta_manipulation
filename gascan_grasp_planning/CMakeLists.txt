cmake_minimum_required(VERSION 2.8.12)
project(gascan_grasp_planning)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        eigen_stl_containers
        geometry_msgs
        roscpp
        rospack
        smpl
        spellbook
        visualization_msgs)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES gascan_grasp_planner
    CATKIN_DEPENDS eigen_stl_containers roscpp spellbook visualization_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(src)

add_library(
    gascan_grasp_planner
    src/grasping/gascan_grasp_planner.cpp)
target_link_libraries(
    gascan_grasp_planner
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_LIBRARIES})

add_library(
    matplotpp
    src/matplotpp/matplotpp.cc
    src/matplotpp/gl2ps.c)
target_link_libraries(matplotpp ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_executable(
    spline_grasp_generator
    src/spline_grasp_generator/spline_grasp_generator.cpp)
target_link_libraries(
    spline_grasp_generator
    ${eigen_conversions_LIBRARIES}
    ${rospack_LIBRARIES}
    ${smpl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${spellbook_LIBRARIES}
    matplotpp)

install(DIRECTORY include/gascan_grasp_planning/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
    TARGETS gascan_grasp_planner matplotpp spline_grasp_generator
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
