#!/bin/bash

## startup ros
roscore&
sleep 2

## upload husky + side-mounted arm + gripper urdf to parameter server
roslaunch hdt upload_andalite.launch sim:=true kinect_sensor:=false&
sleep 1

## bringup simulated hdt arm and ros service for moving the arm; also provides /joint_states
roslaunch hdt hdt_arm_driver.launch sim:=true&
sleep 1

## bring up fake joint states for husky and the fake gripper
roslaunch hdt husky_simulator.launch&
sleep 1
roslaunch hdt gripper_simulator.launch&
sleep 1

## translate joint states to tf frames (make sure you uploaded the urdf from before)
rosrun robot_state_publisher robot_state_publisher&
sleep 1

rosrun rviz rviz

