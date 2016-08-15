add_library(
    robotiq_common
    gripper_model.cpp
    gripper_msgs.cpp
    gripper_status.cpp
    gripper_connection.cpp
    gripper_interface.cpp)

add_executable(
    robotiq_simulator
    robotiq_simulator.cpp
    gripper_simulator.cpp
    gripper_server.cpp)
target_link_libraries(
    robotiq_simulator
    ${Boost_LIBRARIES}
    pthread
    robotiq_common)

add_executable(robotiq_controller robotiq_controller.cpp)
target_link_libraries(
    robotiq_controller
    ${Boost_LIBRARIES}
    pthread robotiq_common)

add_executable(gripper_command_action gripper_command_action.cpp)
target_link_libraries(
    gripper_command_action
    ${Boost_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    robotiq_common)

add_executable(gripper_command_action_client gripper_command_action_client.cpp)
target_link_libraries(
    gripper_command_action_client
    ${Boost_LIBRARIES}
    ${actionlib_LIBRARIES}
    robotiq_common)
