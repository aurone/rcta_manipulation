add_library(hdt_arm_planning HDTRobotModel.cpp)
target_link_libraries(hdt_arm_planning ${catkin_LIBRARIES} hdt_robot_model)

add_executable(hdt_manipulation_test hdt_manipulation_test.cpp)
target_link_libraries(hdt_manipulation_test hdt_arm_planning ${catkin_LIBRARIES})

add_executable(hdt_arm_planning_node hdt_arm_planning_node.cpp ArmPlanningNode.cpp JointInterpolationPathGenerator.cpp)
target_link_libraries(hdt_arm_planning_node hdt_arm_planning ${catkin_LIBRARIES} hdt_robot_model msg_utils)

install(
    TARGETS hdt_arm_planning hdt_arm_planning_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
