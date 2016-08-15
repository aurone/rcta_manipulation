add_executable(
    hdt_train_object_pose_estimation
    hdt_train_object_pose_estimation.cpp)
target_link_libraries(
    hdt_train_object_pose_estimation
    ${roscpp_LIBRARIES}
    ${pr2_vfh_database_LIBRARIES})

install(
    TARGETS hdt_train_object_pose_estimation
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
