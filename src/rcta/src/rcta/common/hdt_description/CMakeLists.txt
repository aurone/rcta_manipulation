add_library(hdt_robot_model RobotModel.cpp)
target_link_libraries(
    hdt_robot_model
    ${leatherman_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${urdf_LIBRARIES}
    ${urdf_parser_LIBRARIES}
    hdt_kinematics)

install(TARGETS hdt_robot_model
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

