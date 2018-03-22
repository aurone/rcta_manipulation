cmake_minimum_required(VERSION 2.8.12)
project(robotiq_controllers)

find_package(Boost REQUIRED COMPONENTS system)
find_package(
    catkin 
    REQUIRED 
    COMPONENTS 
        actionlib 
        control_msgs 
        roscpp 
        sensor_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES robotiq_c85_model)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(src)

# data structures to model the robotiq c85 gripper
add_library(robotiq_c85_model src/gripper_model/gripper_model.cpp)

# library for communicating with the robotiq c85 gripper
# exports boost asio and robotiq_c85_model libs
add_library(
    robotiq_c85_comms
    src/gripper_comms/gripper_msgs.cpp
    src/gripper_comms/gripper_status.cpp
    src/gripper_comms/gripper_connection.cpp
    src/gripper_comms/gripper_interface.cpp)
target_link_libraries(robotiq_c85_comms robotiq_c85_model ${Boost_LIBRARIES})

# ad-hoc simulator to mimic the comms protocol of the live robotiq c85 gripper
add_executable(
    robotiq_simulator
    src/robotiq_simulator/robotiq_simulator.cpp
    src/robotiq_simulator/gripper_simulator.cpp
    src/robotiq_simulator/gripper_server.cpp)
target_link_libraries(
    robotiq_simulator
    ${Boost_LIBRARIES}
    pthread
    robotiq_c85_comms)

# cli teleop control of the gripper
add_executable(
    robotiq_controller 
    src/robotiq_controller/robotiq_controller.cpp)
target_link_libraries(
    robotiq_controller
    ${Boost_LIBRARIES}
    pthread
    robotiq_c85_comms)

# ROS driver for the Robotiq C85 gripper
# Provides:
# * "joint_states" (sensor_msgs/JointState) published at 30 Hz
# * "gripper_controller/gripper_command_action" (control_msgs/GripperCommandAction) action server
add_executable(
    gripper_command_action
    src/gripper_command_action_server/gripper_command_action.cpp)
target_link_libraries(
    gripper_command_action
    ${Boost_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    robotiq_c85_model
    robotiq_c85_comms)

# test action client that opens the gripper
add_executable(
    gripper_command_action_client 
    src/gripper_command_action_client/gripper_command_action_client.cpp)
target_link_libraries(
    gripper_command_action_client
    ${Boost_LIBRARIES}
    ${actionlib_LIBRARIES}
    robotiq_c85_comms)

# TODO: install config and launch directories

install(
    TARGETS
        robotiq_c85_model 
        robotiq_c85_comms 
        robotiq_simulator 
        robotiq_controller
        gripper_command_action
        gripper_command_action_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

