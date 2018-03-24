#include <Eigen/Dense>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <smpl/angles.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

FakeMarkerPublisher::FakeMarkerPublisher() :
    nh_(),
    ph_("~"),
    markers_pub_(),
    listener_(),
    broadcaster_(),
    camera_frame_(),
    wrist_frame_("arm_7_gripper_lift_link")
{
}

bool FakeMarkerPublisher::initialize()
{
    const std::string marker_topic = "ar_pose_marker";
    markers_pub_ = nh_.advertise<ar_track_alvar_msgs::AlvarMarkers>(marker_topic, 1);
    return true;
}

int FakeMarkerPublisher::run()
{
    if (!ph_.getParam("camera_frame", camera_frame_)) {
        ROS_ERROR("Failed to retrieve 'camera_frame' from the param server");
        return FAILED_TO_INITIALIZE;
    }

    double wrist_to_marker_x = 0;
    double wrist_to_marker_y = 0;
    double wrist_to_marker_z = 0;
    double wrist_to_marker_R = 0;
    double wrist_to_marker_P = 0;
    double wrist_to_marker_Y = 0;

    ph_.param("marker_to_link_x", wrist_to_marker_x, 0.0);
    ph_.param("marker_to_link_y", wrist_to_marker_y, 0.0);
    ph_.param("marker_to_link_z", wrist_to_marker_z, 0.0);
    ph_.param("marker_to_link_roll_deg", wrist_to_marker_R, 0.0);
    ph_.param("marker_to_link_pitch_deg", wrist_to_marker_P, 0.0);
    ph_.param("marker_to_link_yaw_deg", wrist_to_marker_Y, 0.0);

    wrist_to_marker_x += 0.02 * (rand() % 101 - 50) / 50.0; //random +/- 5cm offset
    wrist_to_marker_y += 0.02 * (rand() % 101 - 50) / 50.0; //random +/- 5cm offset
    wrist_to_marker_z += 0.02 * (rand() % 101 - 50) / 50.0; //random +/- 5cm offset

    Eigen::Affine3d wrist_to_marker_transform =
                    Eigen::Translation3d(wrist_to_marker_x, wrist_to_marker_y, wrist_to_marker_z) *
                    Eigen::AngleAxisd(sbpl::angles::to_radians(wrist_to_marker_Y), Eigen::Vector3d(0, 0, 1)) *
                    Eigen::AngleAxisd(sbpl::angles::to_radians(wrist_to_marker_P), Eigen::Vector3d(0, 1, 0)) *
                    Eigen::AngleAxisd(sbpl::angles::to_radians(wrist_to_marker_R), Eigen::Vector3d(1, 0, 0));

    ros::Rate loop_rate(10.0);
    int marker_seqno = 0;
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();

        ar_track_alvar_msgs::AlvarMarkers markers;
        markers.header.frame_id = camera_frame_;
        markers.header.stamp = now;
        markers.header.seq = marker_seqno;

        ar_track_alvar_msgs::AlvarMarker marker;
        marker.header.frame_id = camera_frame_;
        marker.header.stamp = now;
        marker.header.seq = marker_seqno;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = wrist_frame_;
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.header.seq = marker_seqno;

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
