#include "ViservoControlExecutor.h"

#include <cmath>
#include <boost/date_time.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/Marker.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/utils/utils.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/utils/RunUponDestruction.h>

ViservoControlExecutor::ViservoControlExecutor() :
    nh_(),
    ph_("~"),
    robot_model_(),
    kdl_robot_model_(),
    action_name_("viservo_command"),
    as_(),
    joint_command_pub_(),
    joint_states_sub_(),
    ar_marker_sub_(),
    current_goal_(),
    last_joint_state_msg_(),
    last_ar_markers_msg_(),
    marker_validity_timeout_(), // read from param server
    wrist_transform_estimate_(),
    max_translational_velocity_mps_(0.05),
    max_rotational_velocity_rps_(sbpl::utils::ToRadians(5.0)),
    max_joint_velocities_rps_(7, sbpl::utils::ToRadians(20.0)),
    listener_(),
    camera_frame_("asus_rgb_frame"),
    wrist_frame_("arm_7_gripper_lift_link"),
    mount_frame_("arm_mount_panel_dummy"),
    attached_marker_()
{
}

int ViservoControlExecutor::run()
{
    as_.reset(new ViservoCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_WARN("Failed to instantiated Viservo Command Action Server");
        return FAILED_TO_INITIALIZE;
    }

    as_->registerGoalCallback(boost::bind(&ViservoControlExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&ViservoControlExecutor::preempt_callback, this));

    if (!download_marker_params()) {
        return FAILED_TO_INITIALIZE;
    }

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

    const std::string& chain_root_link = "arm_mount_panel_dummy"; //robot_model_->joint_names().front();
    const std::string& chain_tip_link = robot_model_->joint_names().back() + "_link";
    const int free_angle = 4;
    kdl_robot_model_.reset(new sbpl_arm_planner::KDLRobotModel(chain_root_link, chain_tip_link, free_angle));
    if (!kdl_robot_model_) {
        ROS_ERROR("Failed to instantiate KDL Robot Model");
    }

    if (!kdl_robot_model_->init(urdf_string, robot_model_->joint_names())) {
        ROS_ERROR("Failed to initialize KDL Robot Model");
        return FAILED_TO_INITIALIZE;
    }

    joint_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    corrected_wrist_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("corrrected_eef_goal", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 1, &ViservoControlExecutor::joint_states_cb, this);
    ar_marker_sub_ = nh_.subscribe("/ar_pose_marker", 1, &ViservoControlExecutor::ar_markers_cb, this);

    ROS_INFO("Starting action server 'viservo_command'...");
    as_->start();
    ROS_INFO("Action server started");

    // overview:
    //
    //     receive a goal pose in the frame of the camera for the tracked marker bundle to achieve
    //     the pose is the pose of the representative marker in the bundle
    //     translate this pose into the wrist frame of the arm
    //     apply P control to determine the direction that the wrist should travel
    //     apply the jacobian in the direction of the error to derive a joint state that moves the arm toward the goal
    //     send that joint state to the arm controller

    // TODO: Attach markers to both the gripper and the forearm
    //       Use the forearm plus the last few joints as one measurement for the current pose of the end effector
    //       Use the gripper markers as another measurement for the current pose of the end effector
    //       Have some filtering mechanism to determine the most confident current pose

    const double rate_hz = 10;
    ros::Rate executive_rate(rate_hz);
    double dt = 1.0 / rate_hz;
    while (ros::ok())
    {
        RunUponDestruction rod([&]() { executive_rate.sleep(); });
        ros::spinOnce();

        if (!as_->isActive()) {
            continue;
        }

        ROS_INFO("Executing viservo control. Current joint state is %s", (bool)last_joint_state_msg_ ? to_string(last_joint_state_msg_->position).c_str() : "null");

        if (as_->isPreemptRequested()) {
            ROS_WARN("Goal preemption currently unimplemented");
        }

        // incorporate the newest marker measurement and estimate the current
        // wrist pose based off of that and the last two joints
        if (!update_wrist_pose_estimate()) {
            ROS_WARN("Failed to update the pose of the wrist");
            continue;
        }

        // 2. check whether the estimated wrist pose has reached the goal within the specified tolerance
        hdt::ViservoCommandResult result;
        if (reached_goal()) {
            ROS_INFO("Wrist has reached goal. Completing action...");
            result.result = hdt::ViservoCommandResult::SUCCESS;
            as_->setSucceeded(result);
            continue;
        }

        // 1. lost track of the marker
        if (lost_marker()) {
            ROS_WARN("Marker has been lost. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::LOST_MARKER;
            as_->setAborted(result);
            continue;
        }

        // 3. check whether the wrist has moved too far from the goal (and the canonical path follower should retry)
        if (moved_too_far()) {
            ROS_INFO("Arm has moved too far from the goal. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::MOVED_TOO_FAR;
            as_->setAborted(result);
            continue;
        }

        Eigen::Affine3d goal_wrist_transform; // in camera frame
        tf::poseMsgToEigen(current_goal_->goal_pose, goal_wrist_transform);

        // transform from the goal to the current pose
        Eigen::Affine3d error = msg_utils::transform_diff(goal_wrist_transform, wrist_transform_estimate_);

        Eigen::Vector3d current_wrist_pos(wrist_transform_estimate_.translation());
        Eigen::Vector3d pos_error(error.translation());

        Eigen::Quaterniond current_quat(wrist_transform_estimate_.rotation());

        // cap the positional error vector by the max translational velocity_mps
        if (pos_error.norm() > max_translational_velocity_mps_ * max_translational_velocity_mps_) {
            pos_error.normalize();
            pos_error *= max_translational_velocity_mps_;
        }

        Eigen::AngleAxisd aa_error(error.rotation());
        double angle_error = aa_error.angle();

        if (angle_error > max_rotational_velocity_rps_) {
            angle_error = clamp(angle_error, -max_rotational_velocity_rps_, max_rotational_velocity_rps_);
        }

        Eigen::Quaterniond delta_quat(Eigen::AngleAxisd(angle_error * dt, aa_error.axis()));

        Eigen::Vector3d target_pos = current_wrist_pos + pos_error * dt;
        Eigen::Quaterniond target_rot = current_quat * delta_quat;

        Eigen::Affine3d camera_to_target_wrist_transform = Eigen::Translation3d(current_wrist_pos) * target_rot;

        // TODO: convert the target transform (which is in the camera frame) to the mount frame

        Eigen::Affine3d mount_to_target_wrist;
        tf::StampedTransform transform;
        const std::string& target_frame = camera_frame_;
        const std::string& source_frame = wrist_frame_;
        try {
            listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("Unable to lookup transform from %s to %s (%s)", source_frame.c_str(), target_frame.c_str(), ex.what());
        }

        hdt::IKSolutionGenerator ikgen = robot_model_->search_all_ik_solutions(
                mount_to_target_wrist, last_joint_state_msg_->position, sbpl::utils::ToRadians(5.0));

        std::vector<double> iksol;
        bool ik_found = ikgen(iksol);
        if (!ik_found) {
            ROS_WARN("Unable to find IK Solution to move the arm towards the goal. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setAborted(result);
            continue;
        }

        // FOR NOW: disallow motions if the joints will not be able to reach
        // their target positions within a reasonable amount of time
        const double angle_threshold_degs = 45;
        for (std::size_t solidx = 0; solidx < iksol.size(); ++solidx) {
            double current_angle = last_joint_state_msg_->position[solidx];
            double solution_angle = iksol[solidx];
            double min_angle = robot_model_->min_limits()[solidx];
            double max_angle = robot_model_->max_limits()[solidx];
            if (sbpl::utils::ShortestAngleDiffWithLimits(solution_angle, current_angle, min_angle, max_angle) >
                sbpl::utils::ToRadians(angle_threshold_degs))
            {
                ROS_ERROR("Next joint state too far away from current joint state (%0.3f degs - %0.3f degs > %0.3f degs)",
                        180.0 * solution_angle / M_PI, 180.0 * current_angle / M_PI, angle_threshold_degs);
                continue;
            }
        }

        // Publish the command
        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.points[0].positions = iksol;
        ROS_INFO("Publishing joint command %s", to_string(traj_cmd.points[0].positions).c_str());
        joint_command_pub_.publish(traj_cmd);
    }

    return SUCCESS;
}

void ViservoControlExecutor::goal_callback()
{
    // TODO: make sure that the markers are in view of the camera or catch this during executive
    current_goal_ = as_->acceptNewGoal();
    ROS_WARN("Received a goal to move the wrist to %s in the camera frame", to_string(current_goal_->goal_pose).c_str());

    // for visualization:
    //     1. estimate the current wrist frame
    //     2.

    geometry_msgs::Vector3 scale;
    scale.x = 0.10;
    scale.y = 0.01;
    scale.z = 0.01;
    visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

    for (auto& marker : triad_marker.markers) {
        marker.header.frame_id = camera_frame_;
    }

    corrected_wrist_goal_pub_.publish(triad_marker);
}

void ViservoControlExecutor::preempt_callback()
{
    // TODO: something important, like preempt the goal
}

void ViservoControlExecutor::joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_DEBUG("Received a Joint State at %s", boost::posix_time::to_simple_string(msg->header.stamp.toBoost()).c_str());

    // filter out joint states that are not for the arm and reorder to canonical joint order
    if (msg_utils::contains_only_joints(*msg, robot_model_->joint_names())) {
        sensor_msgs::JointState::Ptr clean_msg(new sensor_msgs::JointState);
        if (!clean_msg) {
            ROS_ERROR("Failed to instantiate Joint State");
            return;
        }

        *clean_msg = *msg;
        if (!msg_utils::reorder_joints(*clean_msg, robot_model_->joint_names())) {
            ROS_ERROR("Failed to reorder joint names to canonical order");
            return;
        }

        last_joint_state_msg_ = msg; // const everywhere else
    }
}

void ViservoControlExecutor::ar_markers_cb(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
    ROS_DEBUG("Received an AR marker message at %s", boost::posix_time::to_simple_string(msg->header.stamp.toBoost()).c_str());
    last_ar_markers_msg_ = msg;
}

bool ViservoControlExecutor::lost_marker()
{
    if (!last_ar_markers_msg_) {
        ROS_WARN("Haven't received a fresh AR marker message");
        return true;
    }

    if (ros::Time::now() > last_ar_markers_msg_->header.stamp + ros::Duration(marker_validity_timeout_)) {
        ROS_WARN("AR Marker has gone stale after %0.3f seconds", marker_validity_timeout_);
        last_ar_markers_msg_.reset();
        return true;
    }
    return false;
}

bool ViservoControlExecutor::reached_goal() const
{
    Eigen::Affine3d goal_transform;
    tf::poseMsgToEigen(current_goal_->goal_pose, goal_transform);
    Eigen::Affine3d diff = msg_utils::transform_diff(goal_transform, wrist_transform_estimate_);

    Eigen::Vector3d pos_diff(diff.translation());
    Eigen::AngleAxisd rot_diff(diff.rotation());

    return fabs(pos_diff(0)) < goal_pos_tolerance_(0) &&
           fabs(pos_diff(1)) < goal_pos_tolerance_(1) &&
           fabs(pos_diff(2)) < goal_pos_tolerance_(2) &&
           fabs(rot_diff.angle()) < goal_rot_tolerance_;
}

bool ViservoControlExecutor::moved_too_far() const
{
    // TODO: save the original position of the arm (end effector/joints) and
    // abort if they don't appear to be progressing towards the given end
    // effector goal
    return false;
}

bool ViservoControlExecutor::update_wrist_pose_estimate()
{
    if (!last_joint_state_msg_ || !last_ar_markers_msg_) {
        ROS_WARN("Have yet to receive a valid Joint State");
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // get the current pose of the marker in the camera frame
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Affine3d camera_to_marker;
    if (!get_tracked_marker_pose(camera_to_marker)) {
        ROS_WARN("Failed to update wrist. last marker message does not contain a pose for the tracked marker");
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // get the offset from the attached link to the wrist frame using forward
    // kinematics
    ////////////////////////////////////////////////////////////////////////////////

    // run kinematics to the attached link
    std::vector<double> joint_vector = last_joint_state_msg_->position;

    KDL::Frame attached_link_frame;
    if (!kdl_robot_model_->computeFK(joint_vector, attached_marker_.attached_link.c_str(), attached_link_frame)) {
        ROS_ERROR("Failed to compute forward kinematics to link '%s'", attached_marker_.attached_link.c_str());
        return false;
    }

    KDL::Frame wrist_frame;
    if (!kdl_robot_model_->computeFK(joint_vector, wrist_frame_, wrist_frame)) {
        ROS_ERROR("Failed to compute forward kinematics to link '%s'", wrist_frame_.c_str());
        return false;
    }

    // convert kdl frames to eigen frames
    double qw, qx, qy, qz;
    attached_link_frame.M.GetQuaternion(qx, qy, qz, qw);
    Eigen::Affine3d root_to_attached_link =
            Eigen::Translation3d(attached_link_frame.p.x(), attached_link_frame.p.y(), attached_link_frame.p.z()) *
            Eigen::Quaterniond(qw, qx, qy, qz);

    wrist_frame.M.GetQuaternion(qx, qy, qz, qw);
    Eigen::Affine3d root_to_wrist_link =
            Eigen::Translation3d(wrist_frame.p.x(), wrist_frame.p.y(), wrist_frame.p.z()) *
            Eigen::Quaterniond(qw, qx, qy, qz);

    // compute attached link -> wrist via frames from forward kinematics
    Eigen::Affine3d attached_link_to_wrist = root_to_attached_link.inverse() * root_to_wrist_link;

    ////////////////////////////////////////////////////////////////////////////////
    // lookup marker -> attached link transform via configured offset
    ////////////////////////////////////////////////////////////////////////////////

    Eigen::Affine3d marker_to_attached_link = attached_marker_.link_to_marker.inverse();

    ////////////////////////////////////////////////////////////////////////////////
    // combine all transforms to get the estimated wrist pose
    ////////////////////////////////////////////////////////////////////////////////

    // combine transforms to get the marker -> wrist transform
    wrist_transform_estimate_ = camera_to_marker * marker_to_attached_link * attached_link_to_wrist;
    return true;
}

bool ViservoControlExecutor::download_marker_params()
{
    double marker_to_link_x;
    double marker_to_link_y;
    double marker_to_link_z;
    double marker_to_link_roll;
    double marker_to_link_pitch;
    double marker_to_link_yaw;

    bool success =
            msg_utils::download_param(ph_, "tracked_marker_id", attached_marker_.marker_id) &&
            msg_utils::download_param(ph_, "tracked_marker_attached_link", attached_marker_.attached_link) &&
            msg_utils::download_param(ph_, "marker_to_link_x", marker_to_link_x) &&
            msg_utils::download_param(ph_, "marker_to_link_y", marker_to_link_y) &&
            msg_utils::download_param(ph_, "marker_to_link_z", marker_to_link_z) &&
            msg_utils::download_param(ph_, "marker_to_link_roll", marker_to_link_roll) &&
            msg_utils::download_param(ph_, "marker_to_link_pitch", marker_to_link_pitch) &&
            msg_utils::download_param(ph_, "marker_to_link_yaw", marker_to_link_yaw) &&
            msg_utils::download_param(ph_, "marker_validity_timeout", marker_validity_timeout_) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_x_m", goal_pos_tolerance_(0)) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_y_m", goal_pos_tolerance_(1)) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_z_m", goal_pos_tolerance_(2)) &&
            msg_utils::download_param(ph_, "goal_rot_tolerance_deg", goal_rot_tolerance_);

    attached_marker_.link_to_marker = Eigen::Affine3d(
            Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
            Eigen::AngleAxisd(marker_to_link_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(marker_to_link_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(marker_to_link_roll, Eigen::Vector3d::UnitX()));


    return success;
}

bool ViservoControlExecutor::get_tracked_marker_pose(Eigen::Affine3d& marker_pose)
{
    if (!last_ar_markers_msg_) {
        return false;
    }

    bool found = false;
    for (const auto& marker : last_ar_markers_msg_->markers) {
        if (marker.id == attached_marker_.marker_id) {
            const std::string& msg_frame = last_ar_markers_msg_->header.frame_id;
            geometry_msgs::PoseStamped marker_in_msg_frame;
            marker_in_msg_frame.header.frame_id = msg_frame;
            marker_in_msg_frame.header.stamp = last_ar_markers_msg_->header.stamp;

            geometry_msgs::PoseStamped marker_in_camera_frame;

            if (msg_frame != camera_frame_) { // :( time to use me some tf
                try {
                    listener_.transformPose(camera_frame_, marker_in_msg_frame, marker_in_camera_frame);
                }
                catch (const tf::TransformException& ex) {
                    ROS_ERROR("Sadness");
                    return false;
                }
            }
            else {
                marker_in_camera_frame = marker_in_msg_frame;
            }

            tf::poseMsgToEigen(marker_in_camera_frame.pose, marker_pose);
            found = true;
            break;
        }
    }

    return found;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viservo_control_executor");
    return ViservoControlExecutor().run();
}
