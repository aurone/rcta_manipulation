cmake_minimum_required(VERSION 2.8.12)
project(roman_simulation)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(
    catkin
    REQUIRED 
    COMPONENTS 
        actionlib 
        eigen_conversions 
        nav_msgs 
        rosbag
        roscpp 
        rospack
        smpl
        spellbook_ros 
        tf
        tf_conversions)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(sdl2 REQUIRED sdl2)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(${sdl2_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(
    depth_sensor_simulator 
    src/depth_sensor_simulator/depth_sensor_simulator.cpp)
target_link_libraries(
    depth_sensor_simulator
    ${rospack_LIBRARIES}
    ${smpl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sdl2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES})

add_executable(
    ground_truth_publisher 
    src/ground_truth_publisher/ground_truth_publisher.cpp)
target_link_libraries(
    ground_truth_publisher
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${tf_conversions_LIBRARIES})

add_executable(
    kinect_simulator 
    src/kinect_simulator/kinect_simulator.cpp)
target_link_libraries(
    kinect_simulator 
    ${roscpp_LIBRARIES})

#add_executable(
#    localization_simulator 
#    src/localization_simulator/localization_simulator.cpp)
#target_link_libraries(
#    localization_simulator
#    ${actionlib_LIBRARIES}
#    ${eigen_conversions_LIBRARIES}
#    ${rosbag_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${spellbook_ros_LIBRARIES}
#    ${tf_LIBRARIES})

add_executable(
    tilt_mount_simulator 
    src/tilt_mount_simulator/tilt_mount_simulator.cpp)
target_link_libraries(
    tilt_mount_simulator 
    ${roscpp_LIBRARIES})

install(
    TARGETS
        depth_sensor_simulator 
        ground_truth_publisher 
        kinect_simulator 
#        localization_simulator 
        tilt_mount_simulator
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
