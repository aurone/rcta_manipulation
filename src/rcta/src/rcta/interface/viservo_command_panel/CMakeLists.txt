qt4_wrap_cpp(
    RCTA_VISERVO_COMMAND_PANEL_MOC_HEADER_SOURCES
    viservo_command_panel.h)
add_library(
    viservo_command_panel_plugin
    ${RCTA_VISERVO_COMMAND_PANEL_MOC_HEADER_SOURCES}
    viservo_command_panel.cpp)
target_link_libraries(
    viservo_command_panel_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${rviz_LIBRARIES}
    ${tf_LIBRARIES})
install(
    TARGETS viservo_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
