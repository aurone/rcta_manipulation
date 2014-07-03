qt4_wrap_cpp(MOC_HEADER_SOURCES PickAndPlacePanel.h)
add_library(hdt_pick_and_place_plugin ${MOC_HEADER_SOURCES} PickAndPlacePanel.cpp)
target_link_libraries(hdt_pick_and_place_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES})

install(TARGETS hdt_pick_and_place_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

