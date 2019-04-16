cmake_minimum_required(VERSION 2.8.12)
project(object_manipulation_planner)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(fcl REQUIRED fcl)

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
        tf
        tf_conversions
        urdf)

find_package(smpl REQUIRED)
find_package(urdfdom REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DSMPL_CONSOLE_ROS)
add_definitions(-DSV_PACKAGE_NAME="object_manipulation_planner")

# object_manip_planner
add_library(
    object_manip_planner
    src/planner/object_manip_checker.cpp
    src/planner/object_manip_heuristic.cpp
    src/planner/object_manip_planner.cpp
    src/planner/object_manip_model.cpp
    src/planner/roman_workspace_lattice_action_space.cpp
    src/planner/roman_object_manip_lattice.cpp)
target_include_directories(object_manip_planner PUBLIC src/planner)
target_include_directories(
    object_manip_planner
    SYSTEM
    PUBLIC ${Boost_INCLUDE_DIRS}
    PUBLIC ${catkin_INCLUDE_DIRS}
    PUBLIC ${EIGEN3_INCLUDE_DIR}
    PUBLIC ${fcl_INCLUDE_DIRS})
target_link_libraries(
    object_manip_planner
    PUBLIC
    ${fcl_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${smpl_moveit_interface_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    smpl::smpl)
install(
    TARGETS object_manip_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# object_manip_planner_node
add_executable(object_manip_planner_node src/nodes/object_manip_planner_node.cpp)
target_include_directories(
    object_manip_planner_node
    SYSTEM PRIVATE
        ${actionlib_INCLUDE_DIRS}
        ${cmu_manipulation_msgs_INCLUDE_DIRS}
        ${control_msgs_INCLUDE_DIRS}
        ${eigen_conversions_INCLUDE_DIRS}
        ${moveit_ros_planning_interface_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${smpl_urdf_robot_model_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS}
        ${tf_conversions_INCLUDE_DIRS})
target_link_libraries(
    object_manip_planner_node
    PRIVATE
        object_manip_planner
        ${actionlib_LIBRARIES}
        ${cmu_manipulation_msgs_LIBRARIES}
        ${control_msgs_LIBRARIES}
        ${moveit_ros_planning_interface_LIBRARIES}
        ${eigen_conversions_LIBRARIES}
        ${tf_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        ${urdfdom_LIBRARIES}
        ${tf_conversions_LIBRARIES}
        profiler)
install(TARGETS object_manip_planner_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# urdf_to_robot
add_executable(urdf_to_robot src/tools/urdf_to_robot.cpp)
target_compile_options(urdf_to_robot PRIVATE -std=c++11)
target_include_directories(urdf_to_robot SYSTEM PRIVATE ${urdf_INCLUDE_DIRS})
target_link_libraries(urdf_to_robot PRIVATE ${urdf_LIBRARIES})

# draw_kinematics
add_executable(draw_kinematics src/tools/draw_kinematics.cpp)
target_include_directories(
    draw_kinematics
    SYSTEM
    PRIVATE
    ${smpl_urdf_robot_model_INCLUDE_DIRS}
    ${moveit_msgs_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${smpl_ros_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS})
target_link_libraries(
    draw_kinematics
    PRIVATE
    ${smpl_urdf_robot_model_LIBRARIES}
    ${moveit_msgs_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    ${urdf_LIBRARIES}
    smpl::smpl)

# call_planner_fcl
add_executable(call_planner_fcl src/call_planner_fcl.cpp)
target_compile_options(call_planner_fcl PRIVATE -std=c++11)
target_include_directories(
    call_planner_fcl
    SYSTEM PRIVATE
    ${smpl_urdf_robot_model_INCLUDE_DIRS}
    ${fcl_INCLUDE_DIRS}
    ${smpl_ros_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${geometric_shapes_INCLUDE_DIRS})
target_link_libraries(
    call_planner_fcl
    PRIVATE
    ${smpl_urdf_robot_model_LIBRARIES}
    ${fcl_LIBRARIES}
    ${smpl_ros_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${geometric_shapes_LIBRARIES})

# test_object_manip_planner
add_executable(test_object_manip_planner src/test/test_object_manip_planner.cpp)
target_include_directories(test_object_manip_planner SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(test_object_manip_planner PRIVATE object_manip_planner ${Boost_LIBRARIES} smpl::smpl)
install(TARGETS test_object_manip_planner RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# simple_crate_planner
add_executable(simple_crate_planner src/nodes/simple_crate_planner.cpp)
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
install(TARGETS simple_crate_planner RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# manipulate_object
add_executable(manipulate_object src/test/manipulate_object.cpp)
target_include_directories(
    manipulate_object
    SYSTEM PRIVATE
        ${cmu_manipulation_msgs_INCLUDE_DIRS}
        ${actionlib_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${object_manip_planner_INCLUDE_DIRS})
target_link_libraries(manipulate_object PRIVATE ${actionlib_LIBRARIES} ${roscpp_LIBRARIES} smpl::smpl)
install(TARGETS manipulate_object RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# door_demonstrator
add_executable(door_demonstrator src/tools/door_demonstrator.cpp src/tools/cabinet_model.cpp)
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
        ${smpl_ros_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        ${urdfdom_LIBRARIES}
        smpl::smpl)
install(TARGETS door_demonstrator RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# translate_demo
add_executable(translate_demo src/tools/translate_demo.cpp)
target_include_directories(translate_demo SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(translate_demo smpl::smpl)
install(TARGETS translate_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# transform_demo
add_executable(transform_demo src/tools/transform_demo.cpp)
target_include_directories(
    transform_demo
    SYSTEM PRIVATE
        ${moveit_core_INCLUDE_DIRS}
        ${moveit_ros_planning_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS})
target_link_libraries(
    transform_demo
    PRIVATE
        ${moveit_core_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES}
        ${roscpp_LIBRARIES}
        smpl::smpl)
install(TARGETS transform_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# make_cabinet_urdf
add_executable(make_cabinet_urdf src/tools/make_cabinet_urdf.cpp src/tools/cabinet_model.cpp)
target_include_directories(make_cabinet_urdf SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# echo_display_traj
add_executable(echo_display_traj src/tools/echo_display_traj.cpp)
target_include_directories(
    echo_display_traj
    SYSTEM
    PRIVATE
        ${roscpp_INCLUDE_DIRS}
        ${moveit_core_INCLUDE_DIRS}
        ${moveit_ros_planning_INCLUDE_DIRS}
        ${moveit_msgs_INCLUDE_DIRS})
target_link_libraries(
    echo_display_traj
    PRIVATE
        ${roscpp_LIBRARIES}
        ${moveit_core_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES})
install(TARGETS echo_display_traj RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(unprefixer src/tools/unprefixer.cpp)
target_compile_options(unprefixer PRIVATE -std=c++11)
target_include_directories(unprefixer SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})
target_link_libraries(unprefixer PRIVATE ${roscpp_LIBRARIES} ${tf_LIBRARIES})

add_executable(rpy_to_quat src/tools/rpy_to_quat.cpp)
target_include_directories(rpy_to_quat SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
install(TARGETS rpy_to_quat RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
