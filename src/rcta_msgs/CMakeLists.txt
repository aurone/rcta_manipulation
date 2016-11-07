cmake_minimum_required(VERSION 2.8.3)
project(rcta_msgs)

find_package(
	catkin 
	REQUIRED
		genmsg
		actionlib
		actionlib_msgs
		geometry_msgs
		octomap_msgs
		nav_msgs
)

# specify action files
add_action_files(
	DIRECTORY action
	FILES
    GraspObjectCommand.action
	RepositionBaseCommand.action
)

# generate action messages
generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	octomap_msgs
	nav_msgs
)

catkin_package()
