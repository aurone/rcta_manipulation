add_definitions(-DROS=1)  #build using ROS libraries

link_directories("/usr/local/esd/lib64")

################################################################################
# Build motion_download_interface_HDT Executable
################################################################################

set(hdt_LIBRARIES HDT InputDevice ManipulatorError ManipulatorInterface ManipulatorParameters ntcan)

add_executable(motion_download_interface_HDT HDTManipulator.cpp motion_download_interface.cpp)
target_link_libraries(motion_download_interface_HDT ncurses ${hdt_LIBRARIES} pthread ${catkin_LIBRARIES})

################################################################################
# Build joint_trajectory_action Executable
################################################################################

add_executable(joint_trajectory_action_HDT joint_trajectory_action.cpp JointTrajectoryExecutor.cpp)
target_link_libraries(joint_trajectory_action_HDT ${Boost_LIBRARIES} ${catkin_LIBRARIES} hdt_robot_model)

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
