add_executable(ground_truth_publisher ground_truth_publisher.cpp)
target_link_libraries(
    ground_truth_publisher
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tf_LIBRARIES}
    ${tf_conversions_LIBRARIES})
