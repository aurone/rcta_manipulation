add_library(hdt_viz hdt_viz.cpp)
target_link_libraries(hdt_viz ${catkin_LIBRARIES} hdt_robot_model hdt_kinematics)

install(TARGETS hdt_viz
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

