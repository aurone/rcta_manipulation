# HDT Manipulation

HDT Manipulation is a ROS catkin package compatible with the ROS Groovy distribution on Ubuntu 12.04 for planning and control of the grasping pipeline on the HDT robot.

## Installation

#### Set up a catkin workspace

Set up a catkin workspace as instructed in the [ROS tutorials](www.ros.org)

Place the hdt repository inside of the catkin workspace.

Ensure that hdt is on your ROS package path:
>rospack find rcta

You should see something similar to:
>/home/_user_/catkin_workspace/src/hdt

#### Download the source dependencies

Source dependencies are managed by wstool through the .rosinstall file located in the hdt package.

Checkout all source dependencies:

> roscd hdt  
> wstool update

This will checkout the source repositories adjacent to hdt:

    catkin_workspace/
        src/
            CMakeLists.txt
            hdt/
            dep1/
            dep2/
            ...
        build/
        devel/


#### Install System Dependencies

Run the following commands to install and setup all system dependencies for hdt and its source dependencies.

> roscd hdt/scripts  
> ./setup_hdt_environment.sh

This will prompt you for your sudo password to add entries to /etc/apt/sources.list.d and /etc/ros/rosdep/sources.list.d.

A source repository listing is created that points to hdt/extern, allowing APT to find the pre-packaged external debians.

A rosdep rule listing is created that points to hdt/rosdep.yaml to resolve rosdep keys for system dependencies.

setup_hdt_environment.sh runs rosdep install on all of the source repositories to install their system dependencies.

#### Build the catkin workspace

Build the catkin workspace containing the HDT package:
> cd /home/_user_/catkin_workspace  
> catkin_make

If everything went smoothly, catkin_make should exit cleanly and all HDT targets should be built and placed in catkin_workspace/devel

## Running HDT Manipulation

#### Simulation

Launch the top-level launch file to bring up the simulated robot:
> roslaunch hdt andalite.launch sim:=true

Launch the planner and trajectory executor:
> roslaunch hdt hdt_arm_planner.launch  
> roslaunch hdt joint_trajectory_action.launch

Launch the visual servoing executor:
> roslaunch hdt viservo_control_executor.launch

Launch fake AR Marker tracking:
> roslaunch hdt attached_markers.launch sim:=true

Launch the grasp object executor:
> roslaunch hdt grasp_object_executor.launch

Run RViz to control the robot:
> rosrun rviz rviz

Control the robot via the Manipulator Command Panel:
> Add the Manipulator Command Panel via Panels > Add New Panel. Select hdt/ManipulatorCommandPanel.  
> In the Manipulator Command Panel, fill out the name of the URDF parameter on the parameter server (likely /robot_description)
> Add an Interactive Markers Display via Displays [Add] > rviz/InteractiveMarkers
> Select hdt_control/update as the Update Topic for the Interactive Markers Display
> Add a MarkerArray Display via Displays [Add] > rviz/MarkerArray
> If not already selected, select /visualization_marker_array as the Marker Topic

#### Live

TODO
