add_executable(
    retrieve_object_simulator
    CollisionModel2.cpp
    RetrieveObjectSimulator.cpp
    retrieve_object_simulator.cpp)
target_link_libraries(
    retrieve_object_simulator
    ${SBPL_LIBRARIES}
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    hdt_robot_model)

