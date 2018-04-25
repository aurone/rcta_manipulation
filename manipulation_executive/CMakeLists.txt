cmake_minimum_required(VERSION 2.8.12)
project(manipulation_executive)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS actionlib cmu_manipulation_msgs roscpp)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(manipulation_executive src/manipulation_executive.cpp)
target_link_libraries(manipulation_executive ${actionlib_LIBRARIES} ${roscpp_LIBRARIES})

