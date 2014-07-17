add_executable(measure_eef_error measure_eef_error.cpp ErrorMeasurementNode.cpp)
target_link_libraries(measure_eef_error ${catkin_LIBRARIES} hdt_robot_model)

add_executable(fake_marker_pub fake_marker_pub.cpp FakeMarkerPublisher.cpp)
target_link_libraries(fake_marker_pub ${catkin_LIBRARIES})

install(
    TARGETS measure_eef_error
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
