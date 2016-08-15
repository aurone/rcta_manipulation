add_executable(measure_eef_error measure_eef_error.cpp ErrorMeasurementNode.cpp)
target_link_libraries(
    measure_eef_error
    ${roscpp_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model)

add_executable(fake_marker_pub fake_marker_pub.cpp FakeMarkerPublisher.cpp)
target_link_libraries(
    fake_marker_pub
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES})

install(
    TARGETS measure_eef_error fake_marker_pub
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
