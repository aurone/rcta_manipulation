cmake_minimum_required(VERSION 2.8)
project(roman_control)

find_package(Boost REQUIRED)

find_package(
    catkin 
    REQUIRED 
    COMPONENTS
        actionlib 
        control_msgs 
        roscpp 
        roman_client_ros_utils
        robotiq_controllers)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

set(COMMON_LIBS 
    ${actionlib_LIBRARIES}
    ${control_msgs_LIBRARIES} 
    ${roscpp_LIBRARIES} 
    ${roman_client_ros_utils_LIBRARIES})

add_executable(
    right_limb_joint_trajectory_controller
    src/right_limb_joint_trajectory_controller.cpp
    src/roman_joint_trajectory_controller.cpp)
target_link_libraries(right_limb_joint_trajectory_controller ${COMMON_LIBS})

add_executable(
    left_limb_joint_trajectory_controller
    src/left_limb_joint_trajectory_controller.cpp
    src/roman_joint_trajectory_controller.cpp)
target_link_libraries(left_limb_joint_trajectory_controller ${COMMON_LIBS})

add_executable(roman_gripper_controller src/roman_gripper_controller.cpp)
target_link_libraries(
    roman_gripper_controller
    ${actionlib_LIBRARIES}
    ${control_msgs_LBIRARIES}
    ${roscpp_LIBRARIES}
    ${roman_client_ros_utils_LIBRARIES}
    ${robotiq_controllers_LIBRARIES})

install(
    TARGETS
        right_limb_joint_trajectory_controller
        left_limb_joint_trajectory_controller
        roman_gripper_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
