#include "FakeMarkerPublisher.h"

#include <ar_track_alvar/AlvarMarkers.h>

FakeMarkerPublisher::FakeMarkerPublisher() :
	nh_(),
	ph_("~"),
	markers_pub_(),
	listener_(),
	broadcaster_(),
	camera_frame_("kinect_rgb_optical_frame"),
	wrist_frame_("arm_7_gripper_lift_link")
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
    if (!ph_.getParam("camera_frame", camera_frame_)) {
        ROS_ERROR("Failed to retrieve 'camera_frame' from the param server");
        return FAILED_TO_INITIALIZE;
    }

	ros::Rate loop_rate(5.0);
	int marker_seqno = 0;
	while (ros::ok()) {
		ros::spinOnce();
		ros::Time now = ros::Time::now();

		ar_track_alvar::AlvarMarkers markers;
		markers.header.frame_id = camera_frame_;
		markers.header.stamp = now;
		markers.header.seq = marker_seqno;

		ar_track_alvar::AlvarMarker marker;
		marker.header.frame_id = camera_frame_;
		marker.header.stamp = now;
		marker.header.seq = marker_seqno++;

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = wrist_frame_;
		pose_stamped.header.stamp = ros::Time(0);
		pose_stamped.header.seq = marker_seqno;
		pose_stamped.pose.position.x = 0.20;
		pose_stamped.pose.position.y = 0.00;
		pose_stamped.pose.position.z = -0.06;
		pose_stamped.pose.orientation.w = 1.0;
		pose_stamped.pose.orientation.x = 0.0;
		pose_stamped.pose.orientation.y = 0.0;
		pose_stamped.pose.orientation.z = 0.0;

		try {
		    geometry_msgs::PoseStamped wrist_pose_camera_frame;
			listener_.transformPose(camera_frame_, pose_stamped, wrist_pose_camera_frame);

			marker.id = 8;
			marker.pose = wrist_pose_camera_frame;

			markers.markers.push_back(marker);
			markers_pub_.publish(markers);

			tf::StampedTransform marker_transform;
			tf::poseMsgToTF(wrist_pose_camera_frame.pose, marker_transform);
			marker_transform.frame_id_ = camera_frame_;
			marker_transform.child_frame_id_ = "ar_marker_8";
			marker_transform.stamp_ = ros::Time::now();
			broadcaster_.sendTransform(marker_transform);
		}
		catch (const tf::TransformException& ex) {
			ROS_WARN("Failed to transform pose from '%s' to '%s' (%s)", pose_stamped.header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
		}

		loop_rate.sleep();
	}

	return SUCCESS;
}
