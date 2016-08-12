
## These files are for planning/hdt_base_planning simulation

# What is inside?
hdt_run.sh: roslaunch essential launch files in the hdt package for hdt_base_planning simulation
hdt_base_planning_demo.sh: establish a series of arbirtary gastank positions for demonstration
hdt_base_planning_shell.sh: command line to publish simulated camera poses (used by hdt_base_planning_node)

# How to run? (in sequential order)
rosrun hdt hdt_run.sh
rosrun hdt hdt_base_planning_demo.sh

