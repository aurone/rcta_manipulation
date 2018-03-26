cmake_minimum_requireD(VERSION 2.8.12)
project(object_detection)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin 
    REQUIRED 
    COMPONENTS 
        actionlib 
        message_filters 
        tf 
        roscpp 
        pr2_vfh_database)

catkin_package()

add_executable(
    hdt_train_object_pose_estimation 
    src/object_pose_estimation_training/hdt_train_object_pose_estimation.cpp)
target_link_libraries(
    hdt_train_object_pose_estimation
    ${roscpp_LIBRARIES}
    ${pr2_vfh_database_LIBRARIES})

add_executable(
    hdt_pick_and_place 
    src/pick_and_place_node/hdt_pick_and_place.cpp)
target_link_libraries(
    hdt_pick_and_place
    ${actionlib_LIBRARIES}
    ${message_filters_LIBRARIES}
    ${pr2_vfh_database_LIBRARIES}
    ${tf_LIBRARIES}
    ${roscpp_LIBRARIES})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    TARGETS hdt_train_object_pose_estimation hdt_pick_and_place
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
