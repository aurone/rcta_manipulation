add_library(hdt_robot_model RobotModel.cpp)
target_link_libraries(hdt_robot_model ${catkin_LIBRARIES} hdt_kinematics)

install(TARGETS hdt_robot_model
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

