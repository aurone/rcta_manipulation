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
        eigen_conversions
        leatherman
        moveit_core
        moveit_planners_sbpl
        moveit_ros_planning
        roscpp
        sbpl_collision_checking
        smpl
        smpl_ros)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(object_manip_planner src/object_manip_planner.cpp)# src/workspace_lattice_egraph.cpp)
target_include_directories(object_manip_planner PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_include_directories(object_manip_planner PUBLIC include)
target_link_libraries(
    object_manip_planner
    ${moveit_planners_sbpl_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${smpl_LIBRARIES}
    ${smpl_ros_LIBRARIES})

add_executable(door_demonstrator src/door_demonstrator.cpp src/cabinet_model.cpp)
target_include_directories(door_demonstrator PUBLIC SYSTEM ${catkin_INCLUDE_DIRS})
target_include_directories(door_demonstrator PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})
target_link_libraries(door_demonstrator ${roscpp_LIBRARIES})
target_link_libraries(door_demonstrator ${eigen_conversions_LIBRARIES})
target_link_libraries(door_demonstrator ${moveit_core_LIBRARIES})
target_link_libraries(door_demonstrator ${moveit_ros_planning_LIBRARIES})
target_link_libraries(door_demonstrator ${smpl_LIBRARIES})

add_executable(translate_demo src/translate_demo.cpp)
target_include_directories(translate_demo PUBLIC SYSTEM ${smpl_INCLUDE_DIRS})
target_include_directories(translate_demo PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})
target_link_libraries(translate_demo ${smpl_LIBRARIES})

add_executable(make_cabinet_urdf src/make_cabinet_urdf.cpp src/cabinet_model.cpp)
target_include_directories(make_cabinet_urdf PUBLIC SYSTEM ${EIGEN3_INCLUDE_DIR})

