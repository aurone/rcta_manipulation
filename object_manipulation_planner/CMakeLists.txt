cmake_minimum_required(VERSION 2.8.12)
project(object_manipulation_planner)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib_msgs
        eigen_conversions
        leatherman
        moveit_core
        smpl_moveit_interface
        message_generation
        moveit_msgs
        moveit_ros_planning
        roscpp
        sbpl_collision_checking
        smpl
        smpl_urdf_robot_model
        smpl_ros)

find_package(urdfdom REQUIRED)

add_action_files(DIRECTORY action FILES ManipulateObject.action)

generate_messages(DEPENDENCIES moveit_msgs)

catkin_package(CATKIN_DEPENDS message_runtime moveit_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DSMPL_CONSOLE_ROS)
add_definitions(-DSV_PACKAGE_NAME="object_manipulation_planner")

add_library(
    object_manip_planner
    src/object_manip_checker.cpp
    src/object_manip_heuristic.cpp
    src/object_manip_planner.cpp
    src/object_manip_model.cpp
    src/roman_workspace_lattice_action_space.cpp
    src/roman_object_manip_lattice.cpp)

target_include_directories(object_manip_planner PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_include_directories(object_manip_planner PUBLIC include)
target_link_libraries(
    object_manip_planner
    ${smpl_moveit_interface_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${smpl_LIBRARIES}
    ${smpl_ros_LIBRARIES})

add_executable(object_manip_planner_node src/object_manip_planner_node.cpp)
target_link_libraries(object_manip_planner_node object_manip_planner)
target_link_libraries(object_manip_planner_node profiler)
add_dependencies(object_manip_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(manipulate_object src/manipulate_object.cpp)
target_include_directories(manipulate_object SYSTEM PRIVATE ${actionlib_INCLUDE_DIRS})
target_include_directories(manipulate_object SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
target_include_directories(manipulate_object SYSTEM PRIVATE ${object_manip_planner_INCLUDE_DIRS})
target_include_directories(manipulate_object SYSTEM PRIVATE ${smpl_INCLUDE_DIRS})
target_link_libraries(manipulate_object PRIVATE ${actionlib_LIBRARIES})
target_link_libraries(manipulate_object PRIVATE ${roscpp_LIBRARIES})
target_link_libraries(manipulate_object PRIVATE ${smpl_LIBRARIES})
add_dependencies(manipulate_object ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(door_demonstrator src/door_demonstrator.cpp src/cabinet_model.cpp)
target_include_directories(door_demonstrator PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_include_directories(door_demonstrator PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})
target_include_directories(door_demonstrator PUBLIC SYSTEM ${urdfdom_INCLUDE_DIRS})
target_include_directories(door_demonstrator PUBLIC SYSTEM ${smpl_urdf_robot_model_INCLUDE_DIRS})
target_link_libraries(door_demonstrator ${roscpp_LIBRARIES})
target_link_libraries(door_demonstrator ${eigen_conversions_LIBRARIES})
target_link_libraries(door_demonstrator ${moveit_core_LIBRARIES})
target_link_libraries(door_demonstrator ${moveit_ros_planning_LIBRARIES})
target_link_libraries(door_demonstrator ${smpl_LIBRARIES})
target_link_libraries(door_demonstrator ${smpl_ros_LIBRARIES})
target_link_libraries(door_demonstrator ${smpl_urdf_robot_model_LIBRARIES})
target_link_libraries(door_demonstrator ${urdfdom_LIBRARIES})

add_executable(translate_demo src/translate_demo.cpp)
target_include_directories(translate_demo PUBLIC SYSTEM ${smpl_INCLUDE_DIRS})
target_include_directories(translate_demo PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})
target_link_libraries(translate_demo ${smpl_LIBRARIES})

add_executable(make_cabinet_urdf src/make_cabinet_urdf.cpp src/cabinet_model.cpp)
target_include_directories(make_cabinet_urdf PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})

add_executable(echo_display_traj src/echo_display_traj.cpp)
target_include_directories(echo_display_traj PRIVATE SYSTEM ${catkin_INCLUDE_DIRS})
target_link_libraries(echo_display_traj PRIVATE ${roscpp_LIBRARIES})
target_link_libraries(echo_display_traj PRIVATE ${moveit_core_LIBRARIES})
target_link_libraries(echo_display_traj PRIVATE ${moveit_ros_planning_LIBRARIES})

