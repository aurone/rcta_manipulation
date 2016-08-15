qt4_wrap_ui(MOC_FORM_HEADERS TemperatureStatusPanelWidget.ui)
qt4_wrap_cpp(MOC_HEADER_SOURCES TemperatureStatusPanel.h)

add_library(
    temperature_status_plugin
    ${MOC_HEADER_SOURCES}
    ${MOC_FORM_HEADERS}
    TemperatureStatusPanel.cpp)
target_link_libraries(
    temperature_status_plugin
    ${QT_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    qwt)
add_dependencies(temperature_status_plugin ${PROJECT_NAME}_generate_messages_cpp)

install(
    TARGETS temperature_status_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

