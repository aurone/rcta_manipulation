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
	int run();

private:

	ros::NodeHandle nh_;
	ros::Publisher markers_pub_;
	tf::TransformListener listener_;
	tf::TransformBroadcaster broadcaster_;
};



#endif
