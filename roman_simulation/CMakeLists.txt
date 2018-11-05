cmake_minimum_required(VERSION 2.8.12)
project(roman_simulation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        controller_manager
        eigen_conversions
        gazebo_ros_control
        hardware_interface
        nav_msgs
        pluginlib
        rosbag
        roscpp
        rospack
        smpl_urdf_robot_model
        spellbook
        tf
        tf_conversions)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)
pkg_check_modules(sdl2 REQUIRED sdl2)
find_package(smpl REQUIRED)
find_package(urdfdom REQUIRED)

catkin_package()

add_executable(
    depth_sensor_simulator
    src/depth_sensor_simulator/depth_sensor_simulator.cpp)
target_compile_options(depth_sensor_simulator PRIVATE -std=c++11)
target_include_directories(
    depth_sensor_simulator
    SYSTEM
    PRIVATE
    ${sdl2_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${moveit_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${sdl2_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIR}
    ${rospack_INCLUDE_DIRS}
    ${smpl_INCLUDE_DIRS}
    ${spellbook_INCLUDE_DIRS})
target_link_libraries(
    depth_sensor_simulator
    PRIVATE
    ${rospack_LIBRARIES}
    ${smpl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sdl2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES})

# A gazebo_ros_control::RobotHWSim plugin for kinematic control of a robot
# within Gazebo
add_library(
    kinematic_robot_hw_sim
    src/kinematic_robot_hw_sim/kinematic_robot_hw_sim.cpp)
target_compile_options(kinematic_robot_hw_sim PRIVATE -std=c++11)
target_include_directories(
    kinematic_robot_hw_sim
    SYSTEM
    PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${gazebo_ros_control_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${smpl_urdf_robot_model_INCLUDE_DIRS}
    ${hardware_interface_INCLUDE_DIRS})
target_link_libraries(
    kinematic_robot_hw_sim
    PRIVATE
    ${GAZEBO_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${gazebo_ros_control_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_urdf_robot_model_LIBRARIES}
    ${hardware_interface_LIBRARIES})
target_compile_options(kinematic_robot_hw_sim PRIVATE ${GAZEBO_CXX_FLAGS})

add_executable(
    ground_truth_publisher
    src/ground_truth_publisher/ground_truth_publisher.cpp)
target_compile_options(ground_truth_publisher PRIVATE -std=c++11)
target_include_directories(
    ground_truth_publisher
    SYSTEM
    PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${eigen_conversions_INCLUDE_DIRS}
    ${gazebo_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
    ${tf_conversions_INCLUDE_DIRS})
target_link_libraries(
    ground_truth_publisher
    PRIVATE
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${tf_conversions_LIBRARIES})

add_executable(
    kinect_simulator
    src/kinect_simulator/kinect_simulator.cpp)
target_compile_options(kinect_simulator PRIVATE -std=c++11)
target_include_directories(
    kinect_simulator
    SYSTEM
    PRIVATE
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(
    kinect_simulator
    PRIVATE
    ${roscpp_LIBRARIES})

add_executable(
    tilt_mount_simulator
    src/tilt_mount_simulator/tilt_mount_simulator.cpp)
target_compile_options(tilt_mount_simulator PRIVATE -std=c++11)
target_include_directories(
    tilt_mount_simulator
    SYSTEM
    PRIVATE
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS})
target_link_libraries(
    tilt_mount_simulator
    PRIVATE
    ${roscpp_LIBRARIES})

add_executable(robot_simulator src/robot_simulator/robot_simulator.cpp)
target_compile_options(robot_simulator PRIVATE -std=c++11)
target_include_directories(
    robot_simulator
    SYSTEM
    PRIVATE
    ${controller_manager_INCLUDE_DIRS}
    ${hardware_interface_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${smpl_urdf_robot_model_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS})
target_link_libraries(
    robot_simulator
    PRIVATE
    ${controller_manager_LIBRARIES}
    ${hardware_interface_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_urdf_robot_model_LIBRARIES}
    ${urdfdom_LIBRARIES})

add_executable(
    trajectory_dispatcher
    src/trajectory_dispatcher/trajectory_dispatcher.cpp)
target_compile_options(trajectory_dispatcher PRIVATE -std=c++11)
target_include_directories(
    trajectory_dispatcher
    SYSTEM
    PRIVATE
    ${actionlib_INCLUDE_DIRS}
    ${control_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS})
target_link_libraries(
    trajectory_dispatcher
    PRIVATE
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES})

add_executable(
    gripper_command_action_server
    src/gripper_command_action_server/gripper_command_action_server.cpp)
target_compile_options(gripper_command_action_server PRIVATE -std=c++11)
target_include_directories(
    gripper_command_action_server
    SYSTEM
    PRIVATE
    ${actionlib_INCLUDE_DIRS}
    ${control_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS})
target_link_libraries(
    gripper_command_action_server
    PRIVATE
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES})

install(
    TARGETS
        depth_sensor_simulator
        ground_truth_publisher
        kinect_simulator
        robot_simulator
        tilt_mount_simulator
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
