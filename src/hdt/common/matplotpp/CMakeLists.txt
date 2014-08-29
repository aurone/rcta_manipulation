add_library(matplotpp matplotpp.cc gl2ps.c)
target_link_libraries(matplotpp ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

