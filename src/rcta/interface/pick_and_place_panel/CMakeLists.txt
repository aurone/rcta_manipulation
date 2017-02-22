qt4_wrap_cpp(MOC_HEADER_SOURCES PickAndPlacePanel.h)
add_library(
    hdt_pick_and_place_plugin
    ${MOC_HEADER_SOURCES}
    PickAndPlacePanel.cpp
    SimpleInteractiveMarker.cpp
    SimpleInteractiveMarkerServer.cpp
    GraspMarkerSelectionMarker.cpp)
target_link_libraries(
    hdt_pick_and_place_plugin
    ${QT_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${pcl_ros_LIBRARIES}
    ${pr2_vfh_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rviz_LIBRARIES}
    ${tf_LIBRARIES}
    robotiq_common)

install(
    TARGETS hdt_pick_and_place_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

