#include <ros/ros.h>
#include "FakeMarkerPublisher.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fake_marker_pub");

	FakeMarkerPublisher fmp;
	if (!fmp.initialize()) {
		ROS_ERROR("Failed to initialize Fake Marker Publisher");
		return 1;
	}

	return fmp.run();
}
