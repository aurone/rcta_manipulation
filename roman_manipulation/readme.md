# Roman Manipulation

This catkin package contains top-level run configurations to start the
manipulation stack on the RoMan platform. There are three primary
configurations available:

## Live Configuration

This is the default configuration intended to be run against the software stack
running on the robot.

## Sim Configuration

This run configuration uses Gazebo as the backend simulator. This configuration
sucks.

## Fake Configuration

This run configuration uses a fake simulator with limited capabilities,
primarily as a quick tool for testing during development. There is no physical
simulation of the robot or the environment, and no simulation of the robot's
sensors. It does provide ideal control of a simulated robot, with internal
state feedback.

