cmake_minimum_required(VERSION 2.8.3)
project(roman_client_ros_utils)

find_package(
  catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    message_generation)

## jpl libraries
#if(DEFINED ENV{RS_LIMB_ROOT})
#  set(JPL_INCLUDE_DIR $ENV{RS_LIMB_ROOT}/include)
#  #MM- JPL Model Manager, LUMP/RSAP - JPL IPC Comms, PLAN - JPL planning
#  find_library(MM_LIBRARY mm $ENV{RS_LIMB_ROOT}/lib)
#  find_library(LUMP_LIBRARY lump $ENV{RS_LIMB_ROOT}/lib)
#  find_library(RSAP_LIBRARY rsap $ENV{RS_LIMB_ROOT}/lib)
#  find_library(PLAN_LIBRARY plan $ENV{RS_LIMB_ROOT}/lib)
#  set(JPL_LIBRARIES ${PLAN_LIBRARY} ${MM_LIBRARY} ${RSAP_LIBRARY} ${LUMP_LIBRARY} )
#else()
#  MESSAGE(FATAL_ERROR "RS_LIMB_ROOT environment variable not defined")
#endif()
#
### jpl src
#if(DEFINED ENV{RS_LIMB_CLIENT_SRC})
#  set(JPL_SRC_DIR $ENV{RS_LIMB_CLIENT_SRC})
#else()
#  MESSAGE(FATAL_ERROR "RS_LIMB_CLIENT_SRC environment variable not defined")
#endif()

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   RomanWaypoint.msg
   RomanState.msg
   RomanFaultState.msg
   RomanSpec.msg
   RomanSpecReply.msg)

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs)

catkin_package(
  INCLUDE_DIRS #include ${JPL_INCLUDE_DIR}
  LIBRARIES #${JPL_LIBRARIES}
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs)

#include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${JPL_INCLUDE_DIR}
#  ${JPL_INCLUDE_DIR}/common
#)
#
#add_executable(
#  roman_client_jpl2ros
#  src/roman_client_jpl2ros.cpp
#  ${JPL_SRC_DIR}/common/elmo_mech_client_pub.c
#  ${JPL_SRC_DIR}/roman_client/roman_mech_client_pub.c)
#target_link_libraries(roman_client_jpl2ros ${catkin_LIBRARIES} ${JPL_LIBRARIES})
#add_dependencies(
#  roman_client_jpl2ros
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS})
#
#add_executable(
#  roman_client_ros2jpl
#  src/roman_client_ros2jpl.cpp
#  ${JPL_SRC_DIR}/common/elmo_mech_client_pub.c
#  ${JPL_SRC_DIR}/roman_client/roman_mech_client_pub.c)
#target_link_libraries(roman_client_ros2jpl ${catkin_LIBRARIES} ${JPL_LIBRARIES})
#add_dependencies(
#  roman_client_ros2jpl
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS})
#
#add_executable(
#  roman_client_ros_test
#  src/roman_client_ros_test.cpp
#  ${JPL_SRC_DIR}/common/elmo_mech_client_pub.c
#  ${JPL_SRC_DIR}/roman_client/roman_mech_client_pub.c)
#target_link_libraries(roman_client_ros_test ${catkin_LIBRARIES} ${JPL_LIBRARIES})
#add_dependencies(
#  roman_client_ros_test
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS})
