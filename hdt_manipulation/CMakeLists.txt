cmake_minimum_required(VERSION 2.8.12)
project(hdt_manipulation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt4 REQUIRED)
find_package(SBPL REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS
        actionlib
        cmu_manipulation_msgs
        eigen_conversions
        grasping_executive
        hdt_control_msgs
        hdt_kinematics
        interactive_markers
        leatherman
        pcl_ros
        robotiq_controllers
        roscpp
        rviz
        smpl
        spellbook_ros
        tf)

catkin_package()

include(${QT_USE_FILE})

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_library(hdt_arm_planning src/HDTRobotModel.cpp)
target_link_libraries(
    hdt_arm_planning
    ${eigen_conversions_LIBRARIES}
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${smpl_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

add_executable(
    retrieve_object_simulator
    src/retrieve_object_simulator/CollisionModel2.cpp
    src/retrieve_object_simulator/RetrieveObjectSimulator.cpp
    src/retrieve_object_simulator/retrieve_object_simulator.cpp)
target_link_libraries(
    retrieve_object_simulator
    ${SBPL_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${hdt_kinematics_LIBRARIES})

add_executable(stow_arm src/stow_arm/stow_arm.cpp)
target_link_libraries(
    stow_arm
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${hdt_kinematics_LIBRARIES}
    ${spellbook_ros_LIBRARIES})
#add_dependencies(stow_arm ${PROJECT_NAME}_generate_messages_cpp)

#qt4_wrap_cpp(MOC_HEADER_SOURCES src/pick_and_place_panel/PickAndPlacePanel.h)
#add_library(
#    hdt_pick_and_place_plugin
#    ${MOC_HEADER_SOURCES}
#    src/pick_and_place_panel/PickAndPlacePanel.cpp
#    src/pick_and_place_panel/SimpleInteractiveMarker.cpp
#    src/pick_and_place_panel/SimpleInteractiveMarkerServer.cpp
#    src/pick_and_place_panel/GraspMarkerSelectionMarker.cpp)
#target_link_libraries(
#    hdt_pick_and_place_plugin
#    ${QT_LIBRARIES}
#    ${actionlib_LIBRARIES}
#    ${eigen_conversions_LIBRARIES}
#    ${interactive_markers_LIBRARIES}
#    ${pcl_ros_LIBRARIES}
#    ${pr2_vfh_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${rviz_LIBRARIES}
#    ${tf_LIBRARIES}
#    ${robotiq_controllers_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS
        hdt_arm_planning
        retrieve_object_simulator
        stow_arm
#        hdt_pick_and_place_plugin
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

