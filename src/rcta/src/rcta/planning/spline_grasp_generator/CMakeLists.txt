add_executable(spline_grasp_generator spline_grasp_generator.cpp SplineVisualizer.cpp)
target_link_libraries(
    spline_grasp_generator
    ${eigen_conversions_LIBRARIES}
    ${rospack_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${GLUT_LIBRARIES}
    matplotpp
    msg_utils)

