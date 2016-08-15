qt4_wrap_cpp(HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES ManipulatorCommandPanel.h)

add_library(
    hdt_command_plugins
    ${HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES}
    ManipulatorCommandPanel.cpp)
target_link_libraries(
    hdt_command_plugins
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${sbpl_geometry_utils_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model
    robotiq_common
    qwt)

install(
    TARGETS hdt_command_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

