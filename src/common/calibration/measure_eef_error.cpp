#include <ros/ros.h>
#include "ErrorMeasurementNode.h"

int main(int argc, char *argv[])
{
	// 1. place an ar marker in the gripper
	// 2. run ar_track_alvar on that marker to measure its location
	// 3. run forward kinematics to find the location of where the marker is placed on the gripper
	// 4. maintain a running estimate of the error between the two measurements, including the average error and variance

	ros::init(argc, argv, "measure_eef_error");

	ErrorMeasurementNode node;
	if (!node.initialize()) {
		ROS_ERROR("Failed to initialize Error Measurement Node");
		return 1;
	}

	return node.run();
}
