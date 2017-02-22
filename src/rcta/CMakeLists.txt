include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${lua_INCLUDE_DIRS})
include_directories(${sdl_INCLUDE_DIRS})
include_directories("/usr/include/qwt")

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${lua_LIBRARY_DIRS})
link_directories(${sdl_LIBRARY_DIRS})

include_directories(common)

add_subdirectory(common)
add_subdirectory(control)
add_subdirectory(perception)
add_subdirectory(planning)
add_subdirectory(simulation)
add_subdirectory(interface)

