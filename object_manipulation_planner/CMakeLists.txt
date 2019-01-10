cmake_minimum_required(VERSION 2.8.12)
project(object_manipulation_planner)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        cmu_manipulation_msgs
        eigen_conversions
        leatherman
        moveit_core
        smpl_moveit_interface
        moveit_msgs
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp
        sbpl_collision_checking
        smpl_urdf_robot_model
        smpl_ros
        tf)

find_package(smpl REQUIRED)
find_package(urdfdom REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DSMPL_CONSOLE_ROS)
add_definitions(-DSV_PACKAGE_NAME="object_manipulation_planner")

# object_manip_planner
add_library(
    object_manip_planner
    src/object_manip_checker.cpp
    src/object_manip_heuristic.cpp
    src/object_manip_planner.cpp
    src/object_manip_model.cpp
    src/roman_workspace_lattice_action_space.cpp
    src/roman_object_manip_lattice.cpp)
target_include_directories(
    object_manip_planner
    SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include)
target_link_libraries(
    object_manip_planner
    PUBLIC
    ${smpl_moveit_interface_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    smpl::smpl
    ${smpl_ros_LIBRARIES})

# object_manip_planner_node
add_executable(object_manip_planner_node src/object_manip_planner_node.cpp)
target_include_directories(
    object_manip_planner_node
    SYSTEM PRIVATE
        ${actionlib_INCLUDE_DIRS}
        ${cmu_manipulation_msgs_INCLUDE_DIRS}
        ${control_msgs_INCLUDE_DIRS}
        ${eigen_conversions_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${smpl_urdf_robot_model_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS})
target_link_libraries(
    object_manip_planner_node
    PRIVATE
        object_manip_planner
        ${actionlib_LIBRARIES}
        ${cmu_manipulation_msgs_LIBRARIES}
        ${control_msgs_LIBRARIES}
        ${eigen_conversions_LIBRARIES}
        ${tf_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        ${urdfdom_LIBRARIES}
        profiler)

# test_object_manip_planner
add_executable(test_object_manip_planner src/test_object_manip_planner.cpp)
target_include_directories(
    test_object_manip_planner
    SYSTEM PRIVATE
        ${Boost_INCLUDE_DIRS})
target_link_libraries(
    test_object_manip_planner
    PRIVATE
        object_manip_planner
        ${Boost_LIBRARIES}
        smpl::smpl)

# simple_crate_planner
add_executable(simple_crate_planner src/simple_crate_planner.cpp)
target_compile_options(simple_crate_planner PRIVATE -std=c++11)
target_include_directories(
    simple_crate_planner
    SYSTEM PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${actionlib_INCLUDE_DIRS}
        ${cmu_manipulation_msgs_INCLUDE_DIRS}
        ${control_msgs_INCLUDE_DIRS}
        ${eigen_conversions_INCLUDE_DIRS}
        ${moveit_ros_planning_interface_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${smpl_urdf_robot_model_INCLUDE_DIRS}
        ${smpl_ros_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS})
target_link_libraries(
    simple_crate_planner
    PRIVATE
        ${actionlib_LIBRARIES}
        ${moveit_ros_planning_interface_LIBRARIES}
        ${cmu_manipulation_msgs_LIBRARIES}
        ${control_msgs_LIBRARIES}
        ${eigen_conversions_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        ${smpl_ros_LIBRARIES}
        ${urdfdom_LIBRARIES}
        smpl::smpl)

# manipulate_object
add_executable(manipulate_object src/manipulate_object.cpp)
target_include_directories(
    manipulate_object
    SYSTEM PRIVATE
        ${cmu_manipulation_msgs_INCLUDE_DIRS}
        ${actionlib_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${object_manip_planner_INCLUDE_DIRS}
        ${smpl_INCLUDE_DIRS})
target_link_libraries(
    manipulate_object
    PRIVATE
        ${actionlib_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${smpl_LIBRARIES})

# door_demonstrator
add_executable(door_demonstrator src/door_demonstrator.cpp src/cabinet_model.cpp)
target_include_directories(
    door_demonstrator
    SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${urdfdom_INCLUDE_DIRS}
        ${smpl_urdf_robot_model_INCLUDE_DIRS})
target_link_libraries(
    door_demonstrator
    PRIVATE
        ${roscpp_LIBRARIES}
        ${eigen_conversions_LIBRARIES}
        ${moveit_core_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES}
        ${smpl_LIBRARIES}
        ${smpl_ros_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        ${urdfdom_LIBRARIES})

# translate_demo
add_executable(translate_demo src/translate_demo.cpp)
target_include_directories(
    translate_demo
    SYSTEM PUBLIC
        ${smpl_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})
target_link_libraries(translate_demo ${smpl_LIBRARIES})

# transform_demo
add_executable(transform_demo src/transform_demo.cpp)
target_include_directories(
    transform_demo
    SYSTEM PRIVATE
        ${moveit_core_INCLUDE_DIRS}
        ${moveit_ros_planning_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${smpl_INCLUDE_DIRS})
target_link_libraries(
    transform_demo
    PRIVATE
        ${moveit_core_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${smpl_LIBRARIES})

# make_cabinet_urdf
add_executable(make_cabinet_urdf src/make_cabinet_urdf.cpp src/cabinet_model.cpp)
target_include_directories(make_cabinet_urdf SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# echo_display_traj
add_executable(echo_display_traj src/echo_display_traj.cpp)
target_include_directories(
    echo_display_traj
    SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})
target_link_libraries(
    echo_display_traj
    PRIVATE
        ${roscpp_LIBRARIES}
        ${moveit_core_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES})

