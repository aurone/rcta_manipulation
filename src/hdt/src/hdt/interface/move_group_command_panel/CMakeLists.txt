qt4_wrap_cpp(
    MOVE_ARM_COMMAND_PANEL_MOC_SOURCES
    JointVariableCommandWidget.h
    MoveGroupCommandPanel.h
    MoveGroupCommandModel.h)

add_library(
    move_group_command_panel_plugin
    ${MOVE_ARM_COMMAND_PANEL_MOC_SOURCES}
    JointVariableCommandWidget.cpp
    MoveGroupCommandPanel.cpp
    MoveGroupCommandModel.cpp)
target_link_libraries(
    move_group_command_panel_plugin
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES})

install(
    TARGETS move_group_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
