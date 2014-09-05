add_executable(hdt_base_planning_node hdt_base_planning.cpp)
#target_link_libraries(hdt_base_planning_node ${catkin_LIBRARIES})
target_link_libraries(hdt_base_planning_node ${catkin_LIBRARIES} hdt_arm_planning)
# add_dependencies(hdt_base_planning hdt_generate_messages_cpp)
  
