qt4_wrap_ui(MOC_FORM_HEADERS TemperatureStatusPanelWidget.ui)
qt4_wrap_cpp(MOC_HEADER_SOURCES TemperatureStatusPanel.h)

add_library(temperature_status_plugin ${MOC_HEADER_SOURCES} ${MOC_FORM_HEADERS} TemperatureStatusPanel.cpp)
target_link_libraries(temperature_status_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES} qwt)

install(TARGETS temperature_status_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

