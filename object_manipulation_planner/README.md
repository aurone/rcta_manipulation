# Object Manipulation Planner

## Overview

This package provides programs and utilities for planning and execution of
constrained manipulation of simple articulated objects using the RoMan
platform.

## Building

Standard catkin build procedure.

## Nodes

### Object Manipulation Planner Node (`object_manip_planner_node`)

#### Overview

`object_manip_planner_node` provides a ROS action to manipulate an articulated object to a desired position. The action executes a sequence of feasible manipulator trajectories and gripper commands, from the current state, to modify the position of the object.

To manipulate an object, `object_manip_planner_node` requires a demonstration of how that object should be manipulated along its degree-of-freedom. A demonstration includes a trajectory for the robot, annotated with the position of the object at each point and the relative pose of the robot with respect to the object. The 'position' of the object is its intrinsic degree-of-freedom; for example, the demonstration could include a sequence of joint position waypoints for a robotic arm to open a cabinet, with the degree-of-freedom of the object representing the angle of the hinge between open and closed.

#### ROS Parameters

The following parameters are explicitly required by this node.

* `robot_description` - The URDF description of the robot
* `~group_name` - The joint group of the robot being planned for
* `~tip_link` - The tool frame of the robot that manipulates the object
* `~planning_frame` - Frame in which all debug visualizations are specified
* `~follow_trajectory_action_name` - The name of an action server that accepts trajectory for the configured joint group
* `~gripper_command_action_name` - The name of an action server that accepts commands for the gripper

The following parameters are used to configure the collision model:

* `~grid/size_x` - The size in x of the 3D occupancy grid used for collision detection
* `~grid/size_y` - The size in y of the 3D occupancy grid used for collision detection
* `~grid/size_z` - The size in z of the 3D occupancy grid used for collision detection
* `~grid/origin_x` - The minimum x coordinate of the 3D occupancy grid used for collision detection
* `~grid/origin_y` - The minimum y coordinate of the 3D occupancy grid used for collision detection
* `~grid/origin_z` - The minimum z coordinate of the 3D occupancy grid used for collision detection
* `~grid/resolution` - The resolution of the 3D occupancy grid used for collision detection
* `~grid/max_dist` - The maximum propagation radius of the distance transform used for collision detection and heuristic computation. Must be greater than the largest radius of the robot's collision model.

The following parameters are used to configure the planner:

* `~demonstrations_path` - Path to a directory containing demonstrations of how to manipulate the target object
* `~ik_group_name` - A sub-group of the joint group being planned for used by the planner for adaptive motions involving IK
* `~use_rotation` - Whether to include the orientation of the base in the heuristic
* `~disc_rotation_heuristic` - Whether to compute the base orientation heuristic using discrete coordinates
* `~rot_db` - Deadband threshold for the base orientation heuristic when using the real-valued orientation
* `~heading_condition` - Mode for determining when to include the base orientation heuristic, corresponding to enum values in object_manip_heuristic.h
* `~heading_thresh` - The threshold for determining when to include the base orientation, when the heading condition uses the real-valued base orientation
* `~disc_position_heuristic` - Whether to compute the base position heuristic using discrete coordinates
* `~pos_db` - Deadband threshold for the base position heuristic when using the real-valued position
* `~rot_weight` - Weight applied to the base orientation heuristic
* `~base_weight` - Weight applied to the base position heuristic
* `~combination` - Method for combining the base pose heuristic with the contact heuristic
* `~w_egraph` - The weight (penalty) applied to heuristic edges that are not from the demonstration
* `~w_heuristic` - The weight applied to the heuristic term in the search
* `~stats_path` - Path to a file to output statistics to; empty if statistics are not to be written to file

The following parameters are inherited/required to use the SCDL library for collision detection.

* `robot_collision_model/*` - A structure containing the configuration of the `SCDL` model of the robot

The following parameters are inherited/required to use a kinematics and trajectory profiling libraries from MoveIt!

* `robot_description_semantic` - The SRDF description of the robot
* `$ik_group_name/*` - parameter struct to determine and configure the plugin used for kinematics. The `kinematics_solver` sub-parameter is required to select a kinematics plugin.
* `joint_limits/*` - parameter struct to override joint velocity limits and provide joint acceleration limits

#### Published Topics

* `rcta_right_robotiq_controller/command (std_msgs/Float64MultiArray)` - The topic to publish specific gripper commands on with the format expected by the RCTA controller for the Robotiq 3-Finger Adaptive Gripper.
* `visualization_markers` - Debug visualizations and trajectory animation (if $execute was set to false)
* `~planned_path (moveit_msgs/DisplayTrajectory)` - The complete joint trajectory

#### Subscribed Topics

* `joint_states (sensor_msgs/JointState)` - Joint states of the robot
* `/tf (tf2_msgs/TFMessage)` - Transform network
* `/tf_static (tf2_msgs/TFMessage)` - Static transform network

#### Required Actions

* `$~follow_trajectory_action_name (control_msgs/FollowJointTrajectory)` - The action server to send trajectories to.
* `$~gripper_command_action_name (control_msgs/GripperCommand)` - The action server to send gripper commands to. A value of 0 is assumed to represent a closed position for the gripper and a value of 0.0841 represents an open position for the gripper. NOTE: This action server is currently replaced by the specific gripper commands publisher.

#### Provided Actions

* `manipulate_object (cmu_manipulation_msgs/ManipulateObject)` - Accepts requests to manipulate a known object.

#### Files

* `$~stats_path` - A text file containing the results of object manipulation requests. If the path is non-empty, the result is appended to the file at the end of each query. For a query, the line appended is currently either the `<planning-time>,fail` for planning failures or `planning-time,execution-time` for successful planning.

### Manipulate Object (`manipulate_object`)

The `manipulate_object` node reads an example object manipulation query from the ROS parameter server and sends the request to a `manipulate_object` action server.

#### ROS Parameters

* `~allowed_planning_time` - The maximum allowed time to search for a feasible solution
* `~execute` - Whether to send the trajectory and gripper command segments for execution
* `~object_goal_position` - The desired joint configuration of the target object. Default value is 1.0
* `~object_pose` - The pose of the known object
* `~object_start_position` - The starting joint configuration of the target object. Default value is 0.0
* `~start_state` - Any modifications or overrides to the current state of the robot

#### Required Actions

* `manipulate_object (object_manipulation_planner/ManipulateObject)` - The server to send the manipulation request to.

### Object Manipulation Planner Test Suite (`test_object_manip_planner`)

`test_object_manip_planner` reads a set of test scenarios, queries the object manipulation planner for each of them, and records statistics to meausre the planner's performance.

The set of test scenarios is represented as a directory on disk containing a file for each scenario. Each scenario is a YAML file that can be uploaded as parameters to the parameter server.

The test program invokes the `manipulate_object` program several times, once on each test scenario. The return code of the `manipulate_object` program is used to determine whether the planner was successful.

#### Command-Line Interface

`test_object_manip_planner <path/to/scenarios/directory>`

### Simple Crate Planner

The `simple_crate_planner` is a simple state machine behavior that opens the crate. Unlike `object_manip_planner`, `simple_crate_planner` requires URDF models for both the robot and the object. It's primary utility is to generate trajectories that can be used as demonstrations to the primary crate planner, which is more general and robust to variations in the environment.

The `simple_crate_planner` state machine roughly consists of a sequence of behaviors: move-to-pregrasp, open-gripper, move-to-grasp, close-gripper, manipulate-object, open-gripper, and release-crate:

For an input pose for the crate object, the behavior determines a grasp pose to grasp the handle with the tool.

The move-to-pregrasp behavior executes a feasible motion for the manipulator to a pre-grasp pose, a pose offset from the grasp pose by a configured amount along the x-axis of the tool frame. The convention is that the x-axis of the tool frame is the approach direction for the tool.

The move-to-grasp behavior executes a motion that moves the manipulator from the pre-grasp pose to the grasp-pose.

The manipulate-object behavior plans and executes a motion for the arm such that the tool pose follows the crate handle through the range of motion of the lid.

The release-crate behavior executes a motion to move the manipulator away from the handle without closing the crate lid.

#### ROS Parameters

* `object_description` - A URDF description of the object
* `~pregrasp_offset` - Offset from the grasp pose to the pre-grasp pose
* `~release_offset` - Offset from the tool pose after manipulation of the object to the release pose

#### Required Actions

* `rcta_right_robotiq_controller/gripper_action (control_msgs/FollowJointTrajectory)`
* `move_group (moveit_msgs/MoveGroup)`

#### Provided Actions

* `manipulate_object (cmu_manipulation_msgs/ManipulateObject)`

#### Subscribed Topics

Required by MoveGroupInterface:

* `joint_states (sensor_msgs/JointState)`
* `/tf (tf2_msgs/TFMessage)`
* `/tf_static (tf2_msgs/TFMessage)`

#### Published Topics

* `attached_collision_object (moveit_msgs/AttachedCollisionObject)`
* `visualization_markers (visualization_msgs/MarkerArray)`

### Door Demonstrator (`door_demonstrator`)

`door_demonstrator` is a utility program for authoring demonstrations of how to manipulate a known object.

The demonstrator utility requires URDF models of the robot and the object, and an inverse kinematics solver for the object to track the motion of the robot's end effector. The motions of the robot and the object are stored into a file to be processed by the object manipulation planner.

The `door_demonstrator` utility requires a URDF/SRDF for the robot so that the user can specify which joints of the robot should be recorded as part of the demonstration, using one of the known joint groups in the SRDF.

The `door_demonstrator` only applies the kinematics solver to reconfigure the object to track the motion of the robot and record the motions, it does not provide any interface for manipulating the robot. Instead, it subscribes to a topic of type `moveit_msgs/RobotState` and uses that state of the robot to determine contact and object configuration. An RViz plugin and interactive marker interface for manipulating the robot can be found in the `smpl_moveit_interface` package, which publishes the required robot state.

#### ROS Parameters

* `~demo_filename` - - The path where to create a file to store the recorded demonstration in
* `~robot_tip_link` - (string) - The name of the robot link to check for contact with the object
* `~object_tip_link` - (string) - The name of the object link  to check for contact with the robot
* `object_description` - (string)
* `~contact_error_z` - (double)
* `~contact_error` - (double)
* `~object_x` - (double)
* `~object_y` - (double)
* `~object_z` - (double)
* `~object_yaw` - (double)
* `~object_pitch` - (double)
* `~object_roll` - (double)
* `~record` - (bool)
* `~object_positions` - ((string) -> double)
* `~robot_record_variables` - ([]string)
* `~object_variables` - ([]string)
* `~normalize` - (bool)

### Launch Files

### object_manip_planner_node.launch

### door_demonstrator.launch

### manipulate_object.launch

### simple_crate_planner.launch

An example launch file `object_manipulation_planner/object_manip_planner_node.launch` is provided. Currently, the only difference between live and sim is the names of the action servers used for execution.

## Development

