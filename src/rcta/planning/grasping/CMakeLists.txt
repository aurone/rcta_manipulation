add_library(gascan_grasp_planner gascan_grasp_planner.cpp)
target_link_libraries(
    gascan_grasp_planner
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES})
