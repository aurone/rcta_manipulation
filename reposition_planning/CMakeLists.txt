cmake_minimum_required(VERSION 2.8.12)
project(reposition_planning)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        eigen_conversions
        grasp_planner_interface
        grasping_executive
        hdt_kinematics
        leatherman
        moveit_core
        moveit_ros_planning
        pluginlib
        rcta_manipulation_common
        robotiq_controllers
        roscpp
        smpl
        smpl_ros
        spellbook_ros
        tf
        tf_conversions)
find_package(sbpl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${urdfdom_INCLUDE_DIRS})
include_directories(${urdfdom_headers_INCLUDE_DIRS})

add_executable(
    reposition_planner_node
    src/reposition_base_executor/RepositionBaseExecutor.cpp
    src/reposition_base_executor/xytheta_collision_checker.cpp)

target_link_libraries(
    reposition_planner_node
    ${SBPL_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${grasp_planner_interface_LIBRARIES}
    ${hdt_kinematics_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${pluginlib_LIBRARIES}
    ${robotiq_controllers_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    ${tf_conversions_LIBRARIES}
    ${urdf_model_LIBRARIES}
    ${urdf_parser_LIBRARIES})

add_executable(
    reposition_base_simulator
    src/reposition_base_simulator/reposition_base_simulator.cpp)
target_link_libraries(
    reposition_base_simulator
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

