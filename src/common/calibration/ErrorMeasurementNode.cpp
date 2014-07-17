#include "ErrorMeasurementNode.h"

#include <eigen_conversions/eigen_msg.h>

ErrorMeasurementNode::ErrorMeasurementNode() :
	nh_(),
	ph_("~"),
	robot_model_(),
	ar_marker_sub_(),
	joint_state_sub_(),
	marker_msgs_(),
	joint_state_msgs_(),
	last_processed_marker_msg_(),
	last_processed_joint_state_msg_(),
	listener_(),
	mount_frame_("arm_mount_panel_dummy"),
	base_frame_("base_link"),
	camera_frame_("kinect_rgb_optical_frame"),
	transforms_initialized_(false),
	base_frame_to_mount_frame_(),
	base_frame_to_camera_frame_(),
	error_x_(0.0),
	error_y_(0.0),
	error_z_(0.0),
	error_roll_(0.0),
	error_pitch_(0.0),
	error_yaw_(0.0)
{
}

bool ErrorMeasurementNode::initialize()
{
	std::string urdf_string;
	if (!nh_.getParam("robot_description", urdf_string)) {
		ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
		return false;
	}

	if (!robot_model_.load(urdf_string)) {
		ROS_ERROR("Failed to load Robot Model from the URDF");
		return false;
	}

	ar_marker_sub_ = nh_.subscribe("ar_pose_marker", 1, &ErrorMeasurementNode::alvar_markers_callback, this);
	joint_state_sub_ = nh_.subscribe("joint_states", 1, &ErrorMeasurementNode::joint_states_callback, this);
	return true;
}

int ErrorMeasurementNode::run()
{
	ros::Rate loop_rate(30.0);
	while (ros::ok()) {
		ros::spinOnce();

		auto buffer_exhausted = [this]() { return this->marker_msgs_.empty() || this->joint_state_msgs_.empty(); };

		// wait until we've received the fixed transforms we're interested in so that we can compare the marker and fk poses
		if (!transforms_initialized_) {
			if (listener_.canTransform(base_frame_, camera_frame_, ros::Time(0)) && listener_.canTransform(base_frame_, mount_frame_, ros::Time(0))) {
				tf::StampedTransform base_to_camera, base_to_mount;
				listener_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), base_to_camera);
				listener_.lookupTransform(base_frame_, mount_frame_, ros::Time(0), base_to_mount);

				// convert tf to Eigen
				base_frame_to_camera_frame_ = Eigen::Translation3d(base_to_camera.getOrigin().x(), base_to_camera.getOrigin().y(), base_to_camera.getOrigin().z()) *
						                      Eigen::Quaterniond(base_to_camera.getRotation().w(), base_to_camera.getRotation().x(), base_to_camera.getRotation().y(), base_to_camera.getRotation().z());
				base_frame_to_mount_frame_ = Eigen::Translation3d(base_to_mount.getOrigin().x(), base_to_mount.getOrigin().y(), base_to_mount.getOrigin().z()) *
						                     Eigen::Quaterniond(base_to_mount.getRotation().w(), base_to_mount.getRotation().x(), base_to_mount.getRotation().y(), base_to_mount.getRotation().z());

				transforms_initialized_ = true;
			}
			loop_rate.sleep();
			continue;
		}

		if (!buffer_exhausted())
		{
			ROS_INFO("Processing %zd marker messages and %zd joint messages", marker_msgs_.size(), joint_state_msgs_.size());
		}

		while (!buffer_exhausted()) {
			ar_track_alvar::AlvarMarkers::ConstPtr next_marker_msg = marker_msgs_.front();
			sensor_msgs::JointState::ConstPtr next_joint_state_msg = joint_state_msgs_.front();

			if (next_joint_state_msg->header.stamp < next_marker_msg->header.stamp) {
				ROS_INFO("  Processing joint message %d (%0.3f < %0.3f)", next_joint_state_msg->header.seq, next_joint_state_msg->header.stamp.toSec(), next_marker_msg->header.stamp.toSec());
				bool can_interp = last_processed_marker_msg_ && !marker_msgs_.empty();
				if (can_interp) {
					double dt = marker_msgs_.front()->header.stamp.toSec() - last_processed_marker_msg_->header.stamp.toSec();
					double alpha = (next_joint_state_msg->header.stamp.toSec() - last_processed_marker_msg_->header.stamp.toSec()) / dt;
					ROS_INFO("  Interpolating marker messages %d and %d @ %0.3f", last_processed_marker_msg_->header.seq, marker_msgs_.front()->header.seq, alpha);
					Eigen::Affine3d interpolated_pose = interpolated_marker_pose(last_processed_marker_msg_, marker_msgs_.front(), alpha);
				}

				last_processed_joint_state_msg_ = next_joint_state_msg;
				joint_state_msgs_.pop_front();
			}
			else {
				ROS_INFO("  Processing marker message %d (%0.3f < %0.3f)", next_marker_msg->header.seq, next_marker_msg->header.stamp.toSec(), next_joint_state_msg->header.stamp.toSec());
				bool can_interp = last_processed_joint_state_msg_ && !joint_state_msgs_.empty();
				if (can_interp) {
					double dt = joint_state_msgs_.front()->header.stamp.toSec() - last_processed_joint_state_msg_->header.stamp.toSec();
					double alpha = (next_marker_msg->header.stamp.toSec() - last_processed_joint_state_msg_->header.stamp.toSec()) / dt;
					ROS_INFO("  Interpolating joint messages %d and %d @ %0.3f", last_processed_joint_state_msg_->header.seq, joint_state_msgs_.front()->header.seq, alpha);
					Eigen::Affine3d interpolated_pose = interpolated_joint_state_pose(last_processed_joint_state_msg_, joint_state_msgs_.front(), alpha);
				}

				last_processed_marker_msg_ = next_marker_msg;
				marker_msgs_.pop_front();
			}
		}

		// clear all except the last message if there's no hope of being able to process them
		if (!last_processed_joint_state_msg_) {
			while (marker_msgs_.size() > 1) {
				marker_msgs_.pop_front();
			}
		}
		if (!last_processed_marker_msg_) {
			while (joint_state_msgs_.size() > 1) {
				joint_state_msgs_.pop_front();
			}
		}

		loop_rate.sleep();
	}
	return 0;
}

void ErrorMeasurementNode::alvar_markers_callback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
	if (msg->header.frame_id != camera_frame_) {
		ROS_WARN("Expected the marker frames to be expressed in the camera frame '%s'", camera_frame_.c_str());
		return;
	}

	if (!marker_msgs_.empty()) {
		// make sure that we're receiving a message that is newer
		if (marker_msgs_.back()->header.stamp > msg->header.stamp) {
			ROS_WARN("Received a message from the past (%0.3f < %0.3f)", msg->header.stamp.toSec(), marker_msgs_.back()->header.stamp.toSec());
			return;
		}
	}

	marker_msgs_.push_back(msg);
}

void ErrorMeasurementNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	// TODO: assert joint order to simplify things later

	if (!joint_state_msgs_.empty()) {
		if (joint_state_msgs_.back()->header.stamp > msg->header.stamp) {
			ROS_WARN("Received a joint state message from the past (%0.3f < %0.3f)", msg->header.stamp.toSec(), joint_state_msgs_.back()->header.stamp.toSec());
			return;
		}
	}

	joint_state_msgs_.push_back(msg);
}

Eigen::Affine3d ErrorMeasurementNode::interpolated_marker_pose(
	const ar_track_alvar::AlvarMarkers::ConstPtr& first,
	const ar_track_alvar::AlvarMarkers::ConstPtr& second,
	double alpha)
{
	assert(!first->markers.empty());
	assert(!second->markers.empty());

	const ar_track_alvar::AlvarMarker& first_marker = first->markers.front();
	const ar_track_alvar::AlvarMarker& second_marker = second->markers.front();

	Eigen::Affine3d first_marker_transform;
	tf::poseMsgToEigen(first_marker.pose.pose, first_marker_transform);

	Eigen::Affine3d second_marker_transform;
	tf::poseMsgToEigen(second_marker.pose.pose, second_marker_transform);

	// interpolate between the two transforms (camera to marker)
	Eigen::Affine3d interpolated_transform = interpolate(first_marker_transform, second_marker_transform, alpha);

	// transform the resulting transform into the base frame via the fixed transform
	return base_frame_to_camera_frame_ * interpolated_transform;
}

Eigen::Affine3d ErrorMeasurementNode::interpolated_joint_state_pose(
	const sensor_msgs::JointState::ConstPtr &first,
	const sensor_msgs::JointState::ConstPtr& second,
	double alpha)
{
	Eigen::Affine3d first_wrist_transform;
	robot_model_.compute_fk(first->position, first_wrist_transform);

	Eigen::Affine3d second_wrist_transform;
	robot_model_.compute_fk(second->position, second_wrist_transform);

	// interpolate between the two frames
	Eigen::Affine3d interpolated_transform = interpolate(first_wrist_transform, second_wrist_transform, alpha);

	return base_frame_to_mount_frame_ * interpolated_transform;
}

Eigen::Affine3d ErrorMeasurementNode::interpolate(const Eigen::Affine3d& a, const Eigen::Affine3d& b, double alpha)
{
	Eigen::Vector3d interp_pos = (1.0 - alpha) * Eigen::Vector3d(a.translation()) + alpha * Eigen::Vector3d(b.translation());
	Eigen::Quaterniond interp_rot; // TODO:
	return Eigen::Translation3d(interp_pos) * interp_rot;
}
