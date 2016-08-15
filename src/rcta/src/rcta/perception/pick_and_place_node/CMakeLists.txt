add_executable(hdt_pick_and_place hdt_pick_and_place.cpp PickAndPlaceNode.cpp)
target_link_libraries(
    hdt_pick_and_place
    ${actionlib_LIBRARIES}
    ${message_filters_LIBRARIES}
    ${pr2_vfh_database_LIBRARIES}
    ${tf_LIBRARIES}
    ${roscpp_LIBRARIES})

install(TARGETS hdt_pick_and_place RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
