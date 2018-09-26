# Object Manipulation Planner

## Overview

This package provides programs and utilities for planning and execution of
constrained manipulation of simple articulated objects using the Roman
platform.

## Building

Standard catkin build procedure.

## Nodes

### Object Manipulation Planner Node (`object_manip_planner`)

`object_manip_planner_node` is an example application that, given the state of
a robot, its environment, demonstrations of how to manipulate an object, and a
goal configuration for that object (e.g. open or closed), computes a feasible
motion plan for the robot that manipulates the object to the goal
configuration.

An example launch file
`object_manipulation_planner/object_manip_planner_node.launch` is provided.
This launch file provides bool arguments `live` and `execute` to toggle whether
running on a live/simulated robot and whether to send the manipulation commands
for execution, respectively. Currently, the only difference between live and
sim is the names of the action servers used for execution.

#### ROS Parameters

The following parameters are explicitly required by this node.

* robot\_description - The URDF description of the robot
* ~group\_name - The joint group of the robot being planned for
* ~tip\_link - The tool frame of the robot that manipulates the object
* ~allowed\_planning\_time - The maximum allowed time to search for a feasible solution
* ~planning\_frame - Frame in which all debug visualizations are specified
* ~object\_start\_position - The starting joint configuration of the target object. Default value is 0.0
* ~object\_goal\_position - The desired joint configuration of the target object. Default value is 1.0
* ~follow\_trajectory\_action\_name - The name of an action server that accepts trajectory for the configured joint group
* ~gripper\_command\_action\_name - The name of an action server that accepts commands for the gripper
* ~execute - Whether to send the trajectory and gripper command segments for execution

The following parameters are used to configure the collision model:

* ~size\_x - The size in x of the 3D occupancy grid used for collision detection
* ~size\_y - The size in x of the 3D occupancy grid used for collision detection
* ~size\_z - The size in x of the 3D occupancy grid used for collision detection
* ~origin\_x - The minimum x coordinate of the 3D occupancy grid used for collision detection
* ~origin\_y - The minimum x coordinate of the 3D occupancy grid used for collision detection
* ~origin\_z - The minimum x coordinate of the 3D occupancy grid used for collision detection
* ~resolution - The resolution of the 3D occupancy grid used for collision detection
* ~max\_dist - The maximum propagation radius of the distance transform used for collision detection and heuristic computation. Must be greater than the largest radius of the robot's collision model.

The following parameters are used to configure the planner:

* ~demonstrations\_path - Path to a directory containing demonstrations of how to manipulate the target object
* ~ik\_group\_name - A sub-group of the joint group being planned for used by the planner for adaptive motions involving IK
* \<TODO: many other hidden parameters\>

The following parameters are inherited/required to use the SCDL library for collision detection.

* robot\_collision\_model/\* - A structure containing the configuration of the `SCDL` model of the robot

The following parameters are inherited/required to use a few libraries from MoveIt!

* robot\_description\_semantic - The SRDF description of the robot
* $ik\_group\_name/\* - parameter struct to determine and configure the plugin used for kinematics. The kinematics\_solver sub-parameter is required to select a kinematics plugin.
* joint\_limits/\* - parameter struct to override joint velocity limits and provide joint acceleration limits

#### Published Topics

* visualization\_markers - Debug visualizations and trajectory animation (if $execute was set to false)
* ~planned\_path (moveit\_msgs/DisplayTrajectory) - The complete joint trajectory

#### Required Actions

* $~follow\_trajectory\_action\_name - The namespace of the `control_msgs/FollowJointTrajectory` action server to send trajectories
* $~gripper\_command\_action\_name  - The namespace of the `control_msgs/GripperCommand` action server to send gripper commands

### Door Demonstrator (`door_demonstrator`)

`door_demonstrator` is a utility program for authoring demonstrations of how to
manipulate a known object. The demonstrator utility requires URDF models of the
robot and the object, and an inverse kinematics solver for the object to track
the motion of the robot's end effector. The motions of the robot and the object
are stored into a file to be processed by the object manipulation planner.

The `door_demonstrator` utility requires a URDF/SRDF for the robot so that
the user can specify which joints of the robot should be recorded as part of
the demonstration, using one of the known joint groups in the SRDF.

The `door_demonstrator` only applies the kinematics solver to reconfigure the
object to track the motion of the robot and record the motions, it does not
provide any interface for manipulating the robot. Instead, it subscribes to a
topic of type `moveit_msgs/RobotState` and uses that state of the robot to
determine contact and object configuration. An RViz plugin and interactive
marker interface for manipulating the robot can be found in the
`smpl_moveit_interface` package, which publishes the required robot state.

