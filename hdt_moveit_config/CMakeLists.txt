cmake_minimum_required(VERSION 2.8.12)
project(hdt_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

