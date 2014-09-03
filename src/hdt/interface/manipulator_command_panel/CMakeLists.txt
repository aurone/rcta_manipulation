qt4_wrap_cpp(HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES ManipulatorCommandPanel.h)

add_library(hdt_command_plugins
            ${HDT_COMMAND_PLUGIN_MOC_HEADER_SOURCES}
            ManipulatorCommandPanel.cpp)
target_link_libraries(hdt_command_plugins ${catkin_LIBRARIES} ${QT_LIBRARIES} hdt_robot_model msg_utils)

install(TARGETS hdt_command_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

