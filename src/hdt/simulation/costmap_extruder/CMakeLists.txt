add_library(costmap_extruder CostmapExtruder.cpp)
target_link_libraries(costmap_extruder ${catkin_LIBRARIES})

add_executable(costmap_extruder_node costmap_extruder.cpp)
target_link_libraries(costmap_extruder_node costmap_extruder msg_utils ${catkin_LIBRARIES})
