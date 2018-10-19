cmake_minimum_required(VERSION 2.8.3)
project(cmu_manipulation_msgs)

find_package(
	catkin
	REQUIRED
		genmsg
		actionlib
		actionlib_msgs
		geometry_msgs
        moveit_msgs
		nav_msgs
		octomap_msgs
        trajectory_msgs
)

add_action_files(
	DIRECTORY action
	FILES
        GraspObjectCommand.action
        Manipulate.action
	    RepositionBaseCommand.action
        ManipulateObject.action
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	octomap_msgs
    moveit_msgs
	nav_msgs
    trajectory_msgs
)

catkin_package()
