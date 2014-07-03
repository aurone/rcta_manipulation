cmake_minimum_required (VERSION 2.6)
project (HDTManipulator)

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS}  -std=c++0x")

link_directories("/usr/local/esd/lib64")

add_executable(HDTManipulator HDTManipulatorTest.cpp)
target_link_libraries(HDTManipulator ncurses InputDevice HDT ManipulatorParameters ManipulatorError ManipulatorInterface ntcan pthread)
