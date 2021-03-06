# RCTA Manipulation

RCTA Manipulation is a set of catkin packages compatible with the ROS Indigo
distribution on Ubuntu 14.04 that provide planning capabilities for
manipulation on RCTA robotic platforms.

## Building

Set up a catkin workspace as instructed in the [ROS tutorials](www.ros.org) or
determine an existing catkin workspace to host the `rcta_manipulation` packages.

Source dependencies are listed in `rcta_manipulation/rcta.rosinstall` and may
be downloaded simultaneously via [wstool](wiki.ros.org/wstool). Alternatively,
you can manually clone all the packages listed in that file and place them
alongside the `rcta_manipulation` package in your catkin workspace.

System dependencies may be installed using [rosdep](wiki.ros.org/rosdep). At
the root of the workspace, executing the following command will prompt to
install all required system dependencies:

```sh
rosdep install --from-paths rcta_manipulation -i -y
```

The recommended way to build the packages in this repository is to use the
`catkin` from the `python-catkin-tools` package. A workspace containing these
packages may be built using the `catkin build` command.

## Running

There are multiple GNU screen configuration scripts in the `roman_manipulation`
package for running the system under a few configurations. The system can be
run under any configuration using rosrun. For example,

```sh
rosrun roman_manipulation screenrc_fake
```

will run the complete system with an offline (fake) simulation of the RoMan
robot.

Executing a configuration script will start a screen session with tabs preloaded
with the required commands. Launch the system by executing the command within
each tab. Note that the default screen accelerator key C-a is modified to C-x.
Ctrl + the arrow keys can be used to cycle through tabs.
