qt4_wrap_cpp(MOC_HEADER_SOURCES controller_status_panel_widget.h)
qt4_wrap_ui(MOC_FORM_HEADERS controller_status_panel_widget.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR}) # required to locate compiled headers for designer forms

# ControllerStatusPanel.cpp
add_library(hdt_controller_plugins ${MOC_HEADER_SOURCES} ${MOC_FORM_HEADERS} controller_status_panel_widget.cpp)
target_link_libraries(hdt_controller_plugins ${catkin_LIBRARIES} ${QT_LIBRARIES} qwt)
add_dependencies(hdt_controller_plugins hdt_generate_messages_cpp)

install(TARGETS hdt_controller_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

