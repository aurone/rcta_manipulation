# RCTA Manipulation

RCTA Manipulation is a set of catkin packages compatible with the ROS Indigo
distribution on Ubuntu 14.04 that provide planning capabilities for object
manipulation.

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
for d in $(find . -name package.xml); do PKG=$(basename $(dirname $d)); rosdep install -i -y $PKG; done
```

Build the workspace as normal with `catkin_make` or `catkin build`.

