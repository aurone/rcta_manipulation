#include "FakeMarkerPublisher.h"

#include <Eigen/Dense>
#include <ar_track_alvar/AlvarMarkers.h>
#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/utils.h>

FakeMarkerPublisher::FakeMarkerPublisher() :
        nh_(),
        ph_("~"),
        markers_pub_(),
        listener_(),
        broadcaster_(),
        camera_frame_(),
        wrist_frame_("arm_7_gripper_lift_link") {

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

    ros::Rate loop_rate(10.0);
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
        marker.header.seq = marker_seqno;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = wrist_frame_;
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.header.seq = marker_seqno;
        Eigen::Affine3d wrist_to_marker_transform =
                Eigen::Translation3d(0.20, 0.0, -0.06) *
                Eigen::AngleAxisd(sbpl::utils::ToRadians(175.0), Eigen::Vector3d(0, 1, 0));

        tf::poseEigenToMsg(wrist_to_marker_transform.inverse(), pose_stamped.pose);

        try {
            geometry_msgs::PoseStamped wrist_pose_camera_frame;
            listener_.transformPose(camera_frame_, pose_stamped, wrist_pose_camera_frame);

            marker.id = 2;
            marker.pose = wrist_pose_camera_frame;

            markers.markers.push_back(marker);
            markers_pub_.publish(markers);

            tf::StampedTransform marker_transform;
            tf::poseMsgToTF(wrist_pose_camera_frame.pose, marker_transform);
            marker_transform.frame_id_ = camera_frame_;
            marker_transform.child_frame_id_ = "ar_marker_" + std::to_string(marker.id);
            marker_transform.stamp_ = ros::Time::now();
            broadcaster_.sendTransform(marker_transform);
        }
        catch (const tf::TransformException& ex) {
            ROS_WARN("Failed to transform pose from '%s' to '%s' (%s)", pose_stamped.header.frame_id.c_str(),
                camera_frame_.c_str(), ex.what());
        }

        marker_seqno++;
        loop_rate.sleep();
    }

    return SUCCESS;
}
