qt4_wrap_cpp(
    MOVE_ARM_COMMAND_PANEL_MOC_SOURCES 
    MoveArmCommandPanel.h
    MoveArmCommandModel.h)

add_library(
    move_arm_command_panel_plugin
    ${MOVE_ARM_COMMAND_PANEL_MOC_SOURCES} 
    MoveArmCommandPanel.cpp
    MoveArmCommandModel.cpp)
target_link_libraries(
    move_arm_command_panel_plugin
    ${catkin_LIBRARIES} ${QT_LIBRARIES})

install(
    TARGETS move_arm_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
