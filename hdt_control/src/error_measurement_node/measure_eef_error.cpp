#include <ros/ros.h>

// standard includes
#include <cmath>
#include <deque>

// system includes
#include <Eigen/Dense>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <eigen_conversions/eigen_msg.h>
#include <hdt_kinematics/RobotModel.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <spellbook/stringifier/stringifier.h>
#include <tf/transform_listener.h>

class ErrorMeasurementNode
{
public:

    ErrorMeasurementNode();

    bool initialize();
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    hdt::RobotModelPtr robot_model_;

    ros::Subscriber ar_marker_sub_;
    ros::Subscriber joint_state_sub_;

    uint32_t tracked_marker_id_;

    std::list<ar_track_alvar_msgs::AlvarMarker::ConstPtr> marker_msgs_;
    std::list<sensor_msgs::JointState::ConstPtr> joint_state_msgs_;

    ar_track_alvar_msgs::AlvarMarker::ConstPtr last_processed_marker_msg_;
    sensor_msgs::JointState::ConstPtr last_processed_joint_state_msg_;

    tf::TransformListener listener_;

    std::string mount_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    std::string gripper_frame_;
    std::string wrist_frame_;

    bool transforms_initialized_;
    Eigen::Affine3d base_frame_to_mount_frame_;
    Eigen::Affine3d base_frame_to_camera_frame_;
    Eigen::Affine3d wrist_frame_to_gripper_frame_;
    Eigen::Affine3d gripper_frame_to_marker_frame_;

    Eigen::Affine3d total_mean_error_;

    Eigen::Vector3d trans_variance_;
    double rot_variance_;

    double p_; // filter/interpolation factor

    int num_measurements_;

    void alvar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    // return the interpolated transform of the marker frame in the common frame
    Eigen::Affine3d interpolated_marker_pose(
        const ar_track_alvar_msgs::AlvarMarker::ConstPtr& first,
        const ar_track_alvar_msgs::AlvarMarker::ConstPtr& second,
        double alpha);

    // return the interpolated transform of the tool frame in the common frame
    Eigen::Affine3d interpolated_joint_state_pose(
        const sensor_msgs::JointState::ConstPtr& first,
        const sensor_msgs::JointState::ConstPtr& second,
        double alpha);

    // linearly interpolate between two transforms
    Eigen::Affine3d interpolate(const Eigen::Affine3d& a, const Eigen::Affine3d& b, double alpha);

    // compute the pose of the tool frame in the common frame given a joint state
    Eigen::Affine3d compute_joint_state_pose(const sensor_msgs::JointState::ConstPtr& msg);

    // compute the pose of the marker frame in the common frame given a joint state
    Eigen::Affine3d compute_marker_pose(const ar_track_alvar_msgs::AlvarMarker::ConstPtr& msg);

    // compute the difference (error) between two poses
    Eigen::Affine3d compute_pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b);

    Eigen::Affine3d tf_to_eigen(const tf::StampedTransform& transform);

    void updateVariance(const Eigen::Affine3d& error);

    double interp_factor();
};

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
    tracked_marker_id_(8),
    listener_(),
    mount_frame_("arm_mount_panel_dummy"),
    base_frame_("base_link"),
    camera_frame_("/kinect_rgb_optical_frame"),
    gripper_frame_("gripper_base"),
    wrist_frame_("arm_7_gripper_lift_link"),
    transforms_initialized_(false),
    base_frame_to_mount_frame_(),
    base_frame_to_camera_frame_(),
    total_mean_error_(Eigen::Affine3d::Identity()),
    trans_variance_(Eigen::Vector3d::Zero()),
    rot_variance_(0.0),
    p_(1.0 / 80.0/*0.1*/),
    num_measurements_(0)
{
}

bool ErrorMeasurementNode::initialize()
{
    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    if (!(robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string))) {
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
    int loop_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (loop_count % 30 == 0) {
            double trans_error = Eigen::Vector3d(total_mean_error_.translation()).norm();
            double rot_error = 2.0 * acos(Eigen::Quaterniond(total_mean_error_.rotation()).w());

            double trans_variance = trans_variance_.norm();
            ROS_INFO("Error is %0.3fcm, %0.3f degrees. Variance: %0.3fcm, %0.3f degrees", 100.0 * trans_error, 180.0 * rot_error / M_PI, trans_variance, rot_variance_/*, to_string(total_mean_error_).c_str()*/);
        }

        // wait until we've received the fixed transforms we're interested in so that we can compare the marker and fk poses
        if (!transforms_initialized_) {
            if (listener_.canTransform(base_frame_, camera_frame_, ros::Time(0)) &&
                listener_.canTransform(base_frame_, mount_frame_, ros::Time(0)) &&
                listener_.canTransform(wrist_frame_, gripper_frame_, ros::Time(0)))
            {
                tf::StampedTransform base_to_camera, base_to_mount, wrist_to_gripper;
                listener_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), base_to_camera);
                listener_.lookupTransform(base_frame_, mount_frame_, ros::Time(0), base_to_mount);
                listener_.lookupTransform(wrist_frame_, gripper_frame_, ros::Time(0), wrist_to_gripper);

                // convert tf to Eigen
                base_frame_to_camera_frame_ = tf_to_eigen(base_to_camera);
                base_frame_to_mount_frame_ = tf_to_eigen(base_to_mount);
                wrist_frame_to_gripper_frame_ = tf_to_eigen(wrist_to_gripper);
                gripper_frame_to_marker_frame_ = Eigen::Translation3d(0.08, 0.0, 0.0); // TODO: measure this for real. Eigen::Affine3d::Identity();

                transforms_initialized_ = true;
            }
            else {
                ROS_WARN("Failed to acquire all fixed transforms");
            }
            loop_rate.sleep();
            continue;
        }

        auto buffer_exhausted = [this]() { return this->marker_msgs_.empty() || this->joint_state_msgs_.empty(); };

        if (!buffer_exhausted()) {
            ROS_DEBUG("Processing %zd marker messages and %zd joint messages", marker_msgs_.size(), joint_state_msgs_.size());
        }

        while (!buffer_exhausted()) {
            ar_track_alvar_msgs::AlvarMarker::ConstPtr next_marker_msg = marker_msgs_.front();
            sensor_msgs::JointState::ConstPtr next_joint_state_msg = joint_state_msgs_.front();

            if (next_joint_state_msg->header.stamp < next_marker_msg->header.stamp) {
                // Process the next joint state message
                ROS_DEBUG("  Processing joint message %d (%0.3f < %0.3f)", next_joint_state_msg->header.seq, next_joint_state_msg->header.stamp.toSec(), next_marker_msg->header.stamp.toSec());
                bool can_interp = last_processed_marker_msg_ && !marker_msgs_.empty();
                if (can_interp) {
                    double dt = marker_msgs_.front()->header.stamp.toSec() - last_processed_marker_msg_->header.stamp.toSec();
                    double alpha = (next_joint_state_msg->header.stamp.toSec() - last_processed_marker_msg_->header.stamp.toSec()) / dt;
                    ROS_DEBUG("  Interpolating marker messages %d and %d @ %0.3f", last_processed_marker_msg_->header.seq, marker_msgs_.front()->header.seq, alpha);

                    Eigen::Affine3d interpolated_pose = interpolated_marker_pose(last_processed_marker_msg_, marker_msgs_.front(), alpha);
                    Eigen::Affine3d joint_state_pose = compute_joint_state_pose(next_joint_state_msg);

                    Eigen::Affine3d error_pose = compute_pose_diff(interpolated_pose, joint_state_pose);
                    total_mean_error_ = interpolate(total_mean_error_, error_pose, interp_factor());

                    ++num_measurements_;

                    updateVariance(error_pose);
                }

                last_processed_joint_state_msg_ = next_joint_state_msg;
                joint_state_msgs_.pop_front();
            }
            else {
                // Process the next marker message
                ROS_DEBUG("  Processing marker message %d (%0.3f < %0.3f)", next_marker_msg->header.seq, next_marker_msg->header.stamp.toSec(), next_joint_state_msg->header.stamp.toSec());
                bool can_interp = last_processed_joint_state_msg_ && !joint_state_msgs_.empty();
                if (can_interp) {
                    double dt = joint_state_msgs_.front()->header.stamp.toSec() - last_processed_joint_state_msg_->header.stamp.toSec();
                    double alpha = (next_marker_msg->header.stamp.toSec() - last_processed_joint_state_msg_->header.stamp.toSec()) / dt;
                    ROS_DEBUG("  Interpolating joint messages %d and %d @ %0.3f", last_processed_joint_state_msg_->header.seq, joint_state_msgs_.front()->header.seq, alpha);
                    Eigen::Affine3d interpolated_pose = interpolated_joint_state_pose(last_processed_joint_state_msg_, joint_state_msgs_.front(), alpha);
                    Eigen::Affine3d marker_pose = compute_marker_pose(next_marker_msg);

                    Eigen::Affine3d error_pose = compute_pose_diff(marker_pose, interpolated_pose);
                    total_mean_error_ = interpolate(total_mean_error_, error_pose, interp_factor());

                    ++num_measurements_;

                    updateVariance(error_pose);
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

        ++loop_count;
        loop_rate.sleep();
    }
    return 0;
}

void ErrorMeasurementNode::alvar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    ar_track_alvar_msgs::AlvarMarker::Ptr interesting_marker(new ar_track_alvar_msgs::AlvarMarker);
    for (const ar_track_alvar_msgs::AlvarMarker& marker : msg->markers) {
        if (marker.id == tracked_marker_id_) {
            if (marker.header.frame_id != camera_frame_) {
                ROS_WARN("Expected the marker frames to be expressed in the camera frame '%s' (was '%s')", camera_frame_.c_str(), marker.header.frame_id.c_str());
                return;
            }
            interesting_marker.reset(new ar_track_alvar_msgs::AlvarMarker);
            if (!interesting_marker) {
                ROS_WARN("Failed to instantiate Alvar Marker");
                return;
            }

            *interesting_marker = marker;
            interesting_marker->header.seq = msg->header.seq;
        }
    }

    if (!interesting_marker) {
        // did not find the marker we're tracking
        return;
    }

    if (!marker_msgs_.empty()) {
        // make sure that we're receiving a message that is newer
        if (marker_msgs_.back()->header.stamp > msg->header.stamp) {
            ROS_WARN("Received a message from the past (%0.3f < %0.3f)", msg->header.stamp.toSec(), marker_msgs_.back()->header.stamp.toSec());
            return;
        }
    }

    marker_msgs_.push_back(interesting_marker);
}

void ErrorMeasurementNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // TODO: assert joint order to simplify things later
    const int num_joints = 7;
    if (msg->name.size() != num_joints|| msg->position.size() != num_joints) {
        return; // only accept joint states from the arm
    }

    if (!joint_state_msgs_.empty()) {
        if (joint_state_msgs_.back()->header.stamp > msg->header.stamp) {
            ROS_WARN("Received a joint state message from the past (%0.3f < %0.3f)", msg->header.stamp.toSec(), joint_state_msgs_.back()->header.stamp.toSec());
            return;
        }
    }

    joint_state_msgs_.push_back(msg);
}

Eigen::Affine3d ErrorMeasurementNode::interpolated_marker_pose(
    const ar_track_alvar_msgs::AlvarMarker::ConstPtr& first,
    const ar_track_alvar_msgs::AlvarMarker::ConstPtr& second,
    double alpha)
{
    return interpolate(compute_marker_pose(first), compute_marker_pose(second), alpha);
}

Eigen::Affine3d ErrorMeasurementNode::interpolated_joint_state_pose(
    const sensor_msgs::JointState::ConstPtr& first,
    const sensor_msgs::JointState::ConstPtr& second,
    double alpha)
{
    return  interpolate(compute_joint_state_pose(first), compute_joint_state_pose(second), alpha);
}

Eigen::Affine3d ErrorMeasurementNode::interpolate(const Eigen::Affine3d& a, const Eigen::Affine3d& b, double alpha)
{
    Eigen::Vector3d interp_pos = (1.0 - alpha) * Eigen::Vector3d(a.translation()) + alpha * Eigen::Vector3d(b.translation());
    Eigen::Quaterniond aq(a.rotation());
    Eigen::Quaterniond bq(b.rotation());
    Eigen::Quaterniond interp_rot = aq.slerp(alpha, bq);
    return Eigen::Translation3d(interp_pos) * interp_rot;
}

Eigen::Affine3d ErrorMeasurementNode::compute_joint_state_pose(const sensor_msgs::JointState::ConstPtr& msg)
{
    Eigen::Affine3d wrist_transform;
    robot_model_->compute_fk(msg->position, wrist_transform);
    // base -> mount * mount -> wrist * wrist -> gripper * gripper -> tool = base -> tool
    return base_frame_to_mount_frame_ * wrist_transform * wrist_frame_to_gripper_frame_ * gripper_frame_to_marker_frame_;
}

Eigen::Affine3d ErrorMeasurementNode::compute_marker_pose(const ar_track_alvar_msgs::AlvarMarker::ConstPtr& msg)
{
    Eigen::Affine3d marker_transform;
    tf::poseMsgToEigen(msg->pose.pose, marker_transform);
    // base -> camera * camera -> marker = base -> marker
    static const Eigen::Affine3d marker_to_tool(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0)));
    return base_frame_to_camera_frame_ * marker_transform * marker_to_tool;
}

Eigen::Affine3d ErrorMeasurementNode::compute_pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
{
    Eigen::Vector3d apos(a.translation());
    Eigen::Vector3d bpos(b.translation());

    Eigen::Quaterniond arot(a.rotation());
    Eigen::Quaterniond brot(b.rotation());

    return Eigen::Affine3d(Eigen::Translation3d(apos - bpos) * arot.inverse() * brot);
}

Eigen::Affine3d ErrorMeasurementNode::tf_to_eigen(const tf::StampedTransform &transform)
{
    return Eigen::Translation3d(
            transform.getOrigin().x(),
            transform.getOrigin().y(),
            transform.getOrigin().z()) *
           Eigen::Quaterniond(
                   transform.getRotation().w(),
                   transform.getRotation().x(),
                   transform.getRotation().y(),
                   transform.getRotation().z());
}

void ErrorMeasurementNode::updateVariance(const Eigen::Affine3d& error)
{
    Eigen::Affine3d variance_diff = compute_pose_diff(error, total_mean_error_);

    double meanRotDiff = 2 * acos(Eigen::Quaterniond(total_mean_error_.rotation()).w());
    double angle = 2 * acos(Eigen::Quaterniond(variance_diff.rotation()).w());
    auto sqrd = [](double d) { return d * d; };
    rot_variance_ = (1.0 - interp_factor()) * rot_variance_ + interp_factor() * sqrd(angle - meanRotDiff);

    Eigen::Vector3d meanTrans(total_mean_error_.translation());
    Eigen::Vector3d transError = variance_diff.translation();
    Eigen::Vector3d transVariance = (transError - meanTrans).array() * (transError - meanTrans).array();
    trans_variance_ = (1.0 - interp_factor()) * trans_variance_ + interp_factor() * transVariance;
}

double ErrorMeasurementNode::interp_factor()
{
    return p_;
//    return 1.0 / (num_measurements_ + 1);
}

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

