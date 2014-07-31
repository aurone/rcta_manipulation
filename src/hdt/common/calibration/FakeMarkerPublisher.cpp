#include "FakeMarkerPublisher.h"

#include <ar_track_alvar/AlvarMarkers.h>

FakeMarkerPublisher::FakeMarkerPublisher() :
	nh_(),
	markers_pub_(),
	listener_(),
	broadcaster_()
{

}

bool FakeMarkerPublisher::initialize()
{
	const std::string marker_topic = "ar_pose_marker";
	markers_pub_ = nh_.advertise<ar_track_alvar::AlvarMarkers>(marker_topic, 1);
	return true;
}

int FakeMarkerPublisher::run()
{
	ros::Rate loop_rate(5.0);
	int marker_seqno = 0;
	while (ros::ok()) {
		ros::spinOnce();
		ros::Time now = ros::Time::now();

		const std::string camera_frame = "kinect_rgb_optical_frame";
		const std::string wrist_frame = "arm_7_gripper_lift_link";

		ar_track_alvar::AlvarMarkers markers;
		markers.header.frame_id = camera_frame;
		markers.header.stamp = now;
		markers.header.seq = marker_seqno;

		ar_track_alvar::AlvarMarker marker;
		marker.header.frame_id = camera_frame;
		marker.header.stamp = now;
		marker.header.seq = marker_seqno++;

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = wrist_frame;
		pose_stamped.header.stamp = ros::Time(0);//now;
		pose_stamped.header.seq = marker_seqno;
		pose_stamped.pose.position.x = 0.0;
		pose_stamped.pose.position.y = 0.0;
		pose_stamped.pose.position.z = 0.0;
		pose_stamped.pose.orientation.w = 1.0;
		pose_stamped.pose.orientation.x = 0.0;
		pose_stamped.pose.orientation.y = 0.0;
		pose_stamped.pose.orientation.z = 0.0;

		try {
			geometry_msgs::PoseStamped wrist_pose_camera_frame;
			listener_.transformPose(camera_frame, pose_stamped, wrist_pose_camera_frame);

			marker.id = 1;
			marker.pose = wrist_pose_camera_frame;

			markers.markers.push_back(marker);
			markers_pub_.publish(markers);
		}
		catch (const tf::TransformException& ex) {
			ROS_WARN("Failed to transform pose from '%s' to '%s' (%s)", pose_stamped.header.frame_id.c_str(), camera_frame.c_str(), ex.what());
		}

		loop_rate.sleep();
	}

	return 0;
}
