add_library(world_model WorldModel.cpp)

add_executable(world_model_node WorldModelNode.cpp world_model_node.cpp)
target_link_libraries(
    world_model_node
    ${actionlib_LIBRARIES}
    ${roscpp_LIBRARIES}
    world_model)

install(TARGETS world_model world_model_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
