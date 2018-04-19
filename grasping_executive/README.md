# Grasp Object Executor

A pipeline for object pickup following object detection and localization,
with the following steps

* Determine a [set of] collision-free grasp pose for the end effector, given the object
* Move the arm to a pre-grasp pose while avoiding obstacles in the environment
* Pre-shape the end effector for the grasp motion
* Move from the pre grasp pose to the grasp pose (no obstacle avoidance necessary)
* Grasp the object by closing the end effector
* Move from the grasp pose back to the pre-grasp pose
* With the object grasped, move the arm to a stow position for carrying the object
* [check that the object has been removed from its detected location]

# Tweaking the Grasping Pipeline

In addition to the parameters specified in `gascan_grasping.yaml`, the grasping
pipeline has additional parameters listed in
`rcta/config/planning/grasp_object_executor.yaml`.

## `stow_positions`

This is a list of stow positions for the arm to move to after it has grasped
the gascan. It will try each one in order until one succeeds. The name field is
arbitrary for logging purposes.

## `use_extrusion_octomap`

This was for the previous demo where we extruded the 2D costmap to use as the
3D octomap. You probably don't need to enable this.

## `object_filter_radius_m`

This is the approximate radius of the gascan, used to check for whether the
gascan exists in the 2D costmap after an attempted pickup. The detection model
applies a gaussian on top of the costmap centered at where the object was
detected, so cells near the gascan's detected pose will contribute more to the
prediction of whether the gascan was removed from the costmap. The radius
determines the variance of the gaussian (somehow, I forget).

## `gas_can_detection_threshold`

This is the threshold at which and above the gascan is believed to have been
picked up. Basically the product of the gaussian described above, and the
occupied cells centered around the gascan's detected pose.

## `skip_viservo`

Whether to skip the visual servo'ing step (currently true for the Roman)


