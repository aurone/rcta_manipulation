#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <sbpl_geometry_utils/utils.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <hdt/common/stringifier/stringifier.h>

/// This node listens to all incoming messages on /ar_pose_marker and publishes only the messages for markers that
/// are currently attached to the robot (dictated by config). It also attempts to fix the AR marker-flipping by
/// snapping the marker to the most likely rotation given the joint state of the arm
class AttachedMarkerFilter
{
public:

    AttachedMarkerFilter() :
        nh_(),
        ph_("~"),
        alvar_markers_sub_(),
        attached_markers_pub_(),
        joint_state_timeout_s_(2.0),
        listener_(),
        broadcaster_()
    {
    }

    bool initialize()
    {
        std::string urdf_string;
        if (!nh_.getParam("robot_description", urdf_string)) {
            ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
            return FAILED_TO_INITIALIZE;
        }

        robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
        if (!robot_model_) {
            ROS_ERROR("Failed to instantiate Robot Model");
            return FAILED_TO_INITIALIZE;
        }

        alvar_markers_sub_ = nh_.subscribe("ar_pose_marker", 5, &AttachedMarkerFilter::alvar_markers_callback, this);
        joint_state_sub_ = nh_.subscribe("joint_states", 5, &AttachedMarkerFilter::joint_states_callback, this);
        attached_markers_pub_ = nh_.advertise<ar_track_alvar_msgs::AlvarMarkers>("attached_ar_pose_marker", 5);
        return download_marker_params();
    }

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    MainResult run()
    {
        if (!initialize()) {
            // errors printed within
            return FAILED_TO_INITIALIZE;
        }

        ros::spin();
        return SUCCESS;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Subscriber alvar_markers_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher attached_markers_pub_;

    struct AttachedMarker
    {
        int marker_id;
        std::string attached_link;
        Eigen::Affine3d link_to_marker;
    };

    std::vector<AttachedMarker> attached_markers_;

    sensor_msgs::JointState::ConstPtr last_joint_state_;
    double joint_state_timeout_s_;

    hdt::RobotModelPtr robot_model_;

    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;

    void alvar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
    {
        if (!last_joint_state_) {
            ROS_WARN("Have yet to receive a valid joint state");
            return;
        }

        // make sure we have a current enough joint state to estimate where the marker should be at
        if (ros::Time::now() > last_joint_state_->header.stamp + ros::Duration(joint_state_timeout_s_)) {
            ROS_WARN("Joint state has gone stale");
            last_joint_state_.reset();
            return;
        }

        Eigen::Affine3d manip_to_ee;
        if (!robot_model_->compute_fk(last_joint_state_->position, manip_to_ee)) {
            ROS_ERROR("Failed to compute forward kinematics for joint state %s", to_string(last_joint_state_->position).c_str());
            return;
        }


        ar_track_alvar_msgs::AlvarMarkers attached_markers;
        for (const auto& marker : msg->markers) {
            if (tracking_marker(marker.id)) {
                // TODO: attempt to fix orientation
                // TODO: don't depend on marker being attached to the last link in the arm

                // get an expected pose for the marker in the frame of the marker (should be the camera frame)

                // get the transform from the marker to the manipulator frame
                const std::string mount_frame = "arm_mount_panel_dummy";
                tf::StampedTransform transform;
                try {
                    listener_.lookupTransform(marker.header.frame_id, mount_frame, ros::Time(0), transform);
                }
                catch (const tf::TransformException& ex) {
                    ROS_ERROR("Unable to lookup transform from %s to %s (%s)", mount_frame.c_str(), marker.header.frame_id.c_str(), ex.what());
                    return;
                }

                Eigen::Affine3d camera_to_mount =
                    Eigen::Translation3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()) *
                    Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

                // get the offset from the attached link to the marker
                Eigen::Affine3d link_to_marker;
                if (!get_link_to_marker(marker.id, link_to_marker)) {
                    ROS_ERROR("No link-to-marker transform available for marker %u even though it's being tracked", marker.id);
                    return;
                }

                // derive an expected pose for the marker in the camera frame
                Eigen::Affine3d camera_to_expected_marker =
                        camera_to_mount * robot_model_->mount_to_manipulator_transform() * manip_to_ee * link_to_marker;

                // compare the actual pose to the expected pose
                Eigen::Affine3d camera_to_actual_marker;
                tf::poseMsgToEigen(marker.pose.pose, camera_to_actual_marker);

                Eigen::Affine3d error = msg_utils::transform_diff(camera_to_expected_marker, camera_to_actual_marker);

                std::vector<Eigen::Affine3d> marker_transform_candidates = {
                        camera_to_actual_marker,
                        camera_to_actual_marker * Eigen::AngleAxisd(sbpl::utils::ToRadians( 90.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
                        camera_to_actual_marker * Eigen::AngleAxisd(sbpl::utils::ToRadians(180.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
                        camera_to_actual_marker * Eigen::AngleAxisd(sbpl::utils::ToRadians(270.0), Eigen::Vector3d(0.0, 0.0, 1.0))
                };

                // get the most likely offset transform candidate
                auto likely_transform = std::min_element(
                        marker_transform_candidates.begin(), marker_transform_candidates.end(),
                        [&](const Eigen::Affine3d& a, const Eigen::Affine3d& b) {
                            return Eigen::AngleAxisd(msg_utils::transform_diff(camera_to_expected_marker, a).rotation()).angle() <
                                   Eigen::AngleAxisd(msg_utils::transform_diff(camera_to_expected_marker, b).rotation()).angle();
                    });

                // replace the pose of this marker with the corrected pose
                ar_track_alvar_msgs::AlvarMarker corrected_marker = marker;
                tf::poseEigenToMsg(*likely_transform, corrected_marker.pose.pose);

                Eigen::Quaterniond likely_quat(likely_transform->rotation());
                Eigen::Vector3d likely_pos(likely_transform->translation());

                tf::Transform likely_tf(tf::Quaternion(likely_quat.x(), likely_quat.y(), likely_quat.z(), likely_quat.w()),
                                        tf::Vector3(likely_pos.x(), likely_pos.y(), likely_pos.z()));

                // broadcast this pose
                const std::string child_frame = std::string("corrected_ar_marker_") + std::to_string(marker.id);
                tf::StampedTransform corrected_tf(likely_tf, marker.header.stamp, marker.header.frame_id, child_frame);
                broadcaster_.sendTransform(corrected_tf);

                attached_markers.markers.push_back(corrected_marker);
            }
        }

        attached_markers_pub_.publish(attached_markers);
    }

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (msg_utils::contains_only_joints(*msg, robot_model_->joint_names())) {
            last_joint_state_ = msg;
        }
    }

    bool tracking_marker(uint32_t marker_id)
    {
        for (const auto& marker : attached_markers_) {
            if (marker.marker_id == marker_id) {
                return true;
            }
        }
        return false;
    }

    bool get_link_to_marker(uint32_t marker_id, Eigen::Affine3d& transform_out)
    {
        for (const auto& marker : attached_markers_) {
            if (marker.marker_id == marker_id) {
                transform_out = marker.link_to_marker;
                return true;
            }
        }
        return false;
    }

    bool download_marker_params()
    {
        double marker_to_link_x;
        double marker_to_link_y;
        double marker_to_link_z;
        double marker_to_link_roll_degs;
        double marker_to_link_pitch_degs;
        double marker_to_link_yaw_degs;

        AttachedMarker attached_marker;

        bool success =
                msg_utils::download_param(ph_, "tracked_marker_id", attached_marker.marker_id) &&
                msg_utils::download_param(ph_, "tracked_marker_attached_link", attached_marker.attached_link) &&
                msg_utils::download_param(ph_, "marker_to_link_x", marker_to_link_x) &&
                msg_utils::download_param(ph_, "marker_to_link_y", marker_to_link_y) &&
                msg_utils::download_param(ph_, "marker_to_link_z", marker_to_link_z) &&
                msg_utils::download_param(ph_, "marker_to_link_roll_deg", marker_to_link_roll_degs) &&
                msg_utils::download_param(ph_, "marker_to_link_pitch_deg", marker_to_link_pitch_degs) &&
                msg_utils::download_param(ph_, "marker_to_link_yaw_deg", marker_to_link_yaw_degs);

        attached_marker.link_to_marker = Eigen::Affine3d(
            Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

        attached_markers_.push_back(std::move(attached_marker));

        if (!success) {
            ROS_WARN("Failed to download marker params");
        }

        return success;
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "attached_marker_filter");
    return AttachedMarkerFilter().run();
}

