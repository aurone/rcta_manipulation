add_library(msg_utils msg_utils.cpp)
target_link_libraries(msg_utils ${catkin_LIBRARIES})
install(TARGETS msg_utils ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
