set(ros_industrial_LIBRARIES simple_message industrial_robot_client)

# These definitions is copied from the CMakeList for the simple_message package.
# TODO: Figure out how to export this definition from simple_message
add_definitions(-DROS=1)  #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)

link_directories("/usr/local/esd/lib64")

################################################################################
# Build motion_download_interface_HDT Executable
################################################################################

set(MOTOMAN_SOURCES motoman/utils.cpp motoman/trajectory_job.cpp motoman/ros_conversion.cpp)
set(hdt_LIBRARIES HDT InputDevice ManipulatorError ManipulatorInterface ManipulatorParameters ntcan)

add_executable(motion_download_interface_HDT
               HDTManipulator.cpp 
               motion_download_interface.cpp 
               joint_trajectory_downloader.cpp
               ${MOTOMAN_SOURCES})


target_link_libraries(motion_download_interface_HDT
                      ncurses 
                      ${hdt_LIBRARIES} 
                      pthread 
                      # ros_industrial doesn't export its libraries to avoid
                      # byte-order clashes between byte-order-dependent libraries
                      ${ros_industrial_LIBRARIES} 
                      ${catkin_LIBRARIES})

################################################################################
# Build joint_trajectory_action Executable
################################################################################

add_executable(joint_trajectory_action_HDT joint_trajectory_action.cpp JointTrajectoryExecutor.cpp)
target_link_libraries(
    joint_trajectory_action_HDT 
    ${ros_industrial_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${catkin_LIBRARIES}
    hdt_robot_model)

################################################################################
# Build hdt_arm_driver Executable
################################################################################

add_executable(hdt_arm_driver
               hdt_arm_driver.cpp
               manipulator_interface_ros.cpp
               manipulator_interface_live_ros.cpp
               manipulator_interface_sim_ros.cpp)
target_link_libraries(hdt_arm_driver ${catkin_LIBRARIES} ${lua_LIBRARIES} ${hdt_LIBRARIES} config_block hdt_robot_model)
add_dependencies(hdt_arm_driver hdt_generate_messages_cpp)
