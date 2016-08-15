qt4_wrap_cpp(TELEOP_PLUGIN_MOC_HEADER_SOURCES teleop_status_panel.h)
qt4_wrap_ui(TELEOP_PLUGIN_MOC_FORM_HEADERS teleop_status_panel.ui)

add_library(
    hdt_arm_teleop_plugin
    ${TELEOP_PLUGIN_MOC_HEADER_SOURCES}
    ${TELEOP_PLUGIN_MOC_FORM_HEADERS}
    teleop_status_panel.cpp)
target_link_libraries(
    hdt_arm_teleop_plugin
    ${QT_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES})

install(
    TARGETS hdt_arm_teleop_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

