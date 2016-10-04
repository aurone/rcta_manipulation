qt4_wrap_cpp(
    RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES
    gripper_command_panel.h)

add_library(
    gripper_command_panel_plugin
    ${RCTA_GRIPPER_COMMAND_PANEL_MOC_HEADER_SOURCES}
    gripper_command_panel.cpp)
target_link_libraries(
    gripper_command_panel_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    robotiq_common)
install(
    TARGETS gripper_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
