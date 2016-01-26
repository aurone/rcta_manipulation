add_executable(spline_grasp_generator spline_grasp_generator.cpp SplineVisualizer.cpp)
target_link_libraries(spline_grasp_generator ${catkin_LIBRARIES} ${GLUT_LIBRARIES} matplotpp msg_utils)

