qt4_wrap_cpp(
    MOVE_ARM_COMMAND_PANEL_MOC_SOURCES 
    JointVariableCommandWidget.h
    MoveItCommandPanel.h
    MoveArmCommandModel.h)

add_library(
    moveit_command_panel_plugin
    ${MOVE_ARM_COMMAND_PANEL_MOC_SOURCES} 
    JointVariableCommandWidget.cpp
    MoveItCommandPanel.cpp
    MoveArmCommandModel.cpp)
target_link_libraries(
    moveit_command_panel_plugin
    ${catkin_LIBRARIES} ${QT_LIBRARIES})

install(
    TARGETS moveit_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
