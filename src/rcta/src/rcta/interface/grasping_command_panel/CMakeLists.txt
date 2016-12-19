qt4_wrap_cpp(HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES grasping_command_panel.h)

add_library(
    grasping_command_panel_plugin
    ${HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES}
    grasping_command_panel.cpp)
target_link_libraries(
    grasping_command_panel_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model
    robotiq_common
    qwt)

install(
    TARGETS grasping_command_panel_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

