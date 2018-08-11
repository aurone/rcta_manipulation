# Object Manipulation Planner

## Overview

This package provides programs and utilities for planning for constrained
manipulation of simple articulated objects on the Roman platform.

## Object Manipulation Planner Node

`object_manip_planner_node` is an example application that, given the state of
a robot, its environment, demonstrations of how to manipulate an object, and a
goal configuration for that object (e.g. open or closed), computes a feasible
motion plan for the robot that manipulates the object to the goal
configuration.

## Door Demonstrator (TODO: Rename)

`door_demonstrator` is a utility program for authoring demonstrations of how to
manipulate a known object. The demonstrator utility requires a model of the
robot and of the object, and an inverse kinematics solver for the object to
reconfigure it to track the motion of the robot's end effector. The motions of
the robot and the object are stored into a file to be processed by the object
manipulation planner.

The `door_demonstrator` utility requires a URDF/SRDF for the object so that
the user can specify which joints of the robot should be recorded as part of
the demonstration, using one of the known joint groups in the SRDF.

The `door_demonstrator` only applies the kinematics solver to reconfigure the
object to track the motion of the robot and record the motions, it does not
provide any interface for manipulating the robot. Instead, it subscribes to a
topic of type `moveit_msgs/RobotState` and uses that state of the robot to
determine contact and object configuration. An RViz plugin and interactive
marker interface for manipulating the robot can be found in the
`smpl_moveit_interface` package, which publishes the required robot state.

