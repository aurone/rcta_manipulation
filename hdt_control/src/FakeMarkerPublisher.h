#ifndef FakeMarkerPublisher_h
#define FakeMarkerPublisher_h

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class FakeMarkerPublisher
{
public:

	FakeMarkerPublisher();

	bool initialize();

	enum RunResult
	{
	    SUCCESS = 0,
	    FAILED_TO_INITIALIZE
	};
	int run();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle ph_;
	ros::Publisher markers_pub_;
	tf::TransformListener listener_;
	tf::TransformBroadcaster broadcaster_;

	std::string camera_frame_;
	std::string wrist_frame_;
};

#endif
