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
    max_rotational_velocity_rps_(sbpl::utils::ToRadians(1.0)),
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

    const std::string& chain_root_link = "arm_mount_panel_dummy";
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
    corrected_wrist_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("corrected_eef_goal", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ViservoControlExecutor::joint_states_cb, this);
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

    const double rate_hz = 10.0;
    ros::Rate executive_rate(rate_hz);
    double dt = 1.0 / rate_hz;
    while (ros::ok())
    {
        RunUponDestruction rod([&]() { executive_rate.sleep(); });
        ros::spinOnce();

        if (!as_->isActive()) {
            continue;
        }

        ROS_INFO("Executing viservo control. Current joint state is %s",
                (bool)last_joint_state_msg_ ? to_string(msg_utils::to_degrees(last_joint_state_msg_->position)).c_str() : "null");

        if (as_->isPreemptRequested()) {
            ROS_WARN("Goal preemption currently unimplemented");
        }

        // incorporate the newest marker measurement and estimate the current
        // wrist pose based off of that and the last two joints
        if (!update_wrist_pose_estimate()) {
            ROS_WARN("Failed to update the pose of the wrist");
            continue;
        }

        tf::StampedTransform transform;
        try {
//            listener_.lookupTransform(mount_frame_, camera_frame_, ros::Time(0), transform);
            listener_.lookupTransform(camera_frame_, mount_frame_, ros::Time(0), transform);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("Unable to lookup transform from %s to %s (%s)", mount_frame_.c_str(), camera_frame_.c_str(), ex.what());
        }

        Eigen::Affine3d camera_to_mount =
            Eigen::Translation3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()) *
            Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

        ROS_WARN("Mount to camera: %s", to_string(camera_to_mount.inverse()).c_str());

        hdt::ViservoCommandResult result;

        if (!robot_model_->compute_fk(last_joint_state_msg_->position, eef_transform_from_joint_state_)) {
            ROS_ERROR("Failed to compute end effector transform from joint state");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setSucceeded(result);
            continue;
        }

        // camera -> wrist = camera -> mount * mount -> wrist
        eef_transform_from_joint_state_ = camera_to_mount * eef_transform_from_joint_state_;

        // 2. check whether the estimated wrist pose has reached the goal within the specified tolerance
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

        double roll, pitch, yaw;

        Eigen::Affine3d goal_wrist_transform; // in camera frame
        tf::poseMsgToEigen(current_goal_->goal_pose, goal_wrist_transform);

        Eigen::Quaterniond goal_quat(goal_wrist_transform.rotation());
        Eigen::Vector3d goal_pos(goal_wrist_transform.translation());
        tf::Transform goalTF(tf::Quaternion(goal_quat.x(), goal_quat.y(), goal_quat.z(), goal_quat.w()), tf::Vector3(goal_pos.x(), goal_pos.y(), goal_pos.z()));
        goalTF.getBasis().getEulerYPR(yaw, pitch, roll, 1);

        // transform from the goal to the current pose
        Eigen::Affine3d error = msg_utils::transform_diff(goal_wrist_transform, wrist_transform_estimate_);

        ROS_INFO("  Goal Wrist Transform (camera frame): %s", to_string(goal_wrist_transform).c_str());
        ROS_INFO("      Euler Angles: r: %0.3f, p: %0.3f, y: %0.3f", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));
        ROS_INFO("  Curr Wrist Transform (camera frame): %s", to_string(wrist_transform_estimate_).c_str());
        ROS_INFO("  Error: %s", to_string(error).c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target position
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Vector3d current_wrist_pos(eef_transform_from_joint_state_.translation());
        Eigen::Vector3d pos_error(error.translation());

        ROS_INFO("    Position Error: %s", to_string(pos_error).c_str());

        // cap the positional error vector by the max translational velocity_mps
        if (pos_error.norm() > max_translational_velocity_mps_ * max_translational_velocity_mps_) {
            pos_error.normalize();
            pos_error *= max_translational_velocity_mps_;
        }

        Eigen::Vector3d target_pos = current_wrist_pos + pos_error * dt;

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target rotation
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::AngleAxisd aa_error(error.rotation());
        double angle_error = aa_error.angle();

        ROS_INFO("    Rotation Error: %0.3f degs about %s", sbpl::utils::ToDegrees(angle_error), to_string(aa_error.axis()).c_str());

        if (fabs(angle_error) > fabs(max_rotational_velocity_rps_)) {
            angle_error = clamp(angle_error, -max_rotational_velocity_rps_, max_rotational_velocity_rps_);
        }

        Eigen::Quaterniond error_quat(error.rotation());
        Eigen::Quaterniond current_quat(eef_transform_from_joint_state_.rotation());

        ROS_INFO("    Wrist Rotation Estimate: %s", to_string(Eigen::AngleAxisd(wrist_transform_estimate_.rotation())).c_str());
        ROS_INFO("    Goal Rotation: %s", to_string(Eigen::AngleAxisd(goal_wrist_transform.rotation())).c_str());
        ROS_INFO("    Error Rotation: %s", to_string(Eigen::AngleAxisd(error_quat)).c_str());
        ROS_INFO("    Error Rotation Inverse: %s", to_string(Eigen::AngleAxisd(error_quat.inverse())).c_str());
        ROS_INFO("    Current Rotation Inverse: %s", to_string(Eigen::AngleAxisd(current_quat)).c_str());
        ROS_INFO("    Current Rotation x Error: %s", to_string(Eigen::AngleAxisd(current_quat * error_quat)).c_str());
        ROS_INFO("    Current Rotation x Error Inverse: %s", to_string(Eigen::AngleAxisd(current_quat * error_quat.inverse())).c_str());
        ROS_INFO("    Error x Current Rotation: %s", to_string(Eigen::AngleAxisd(error_quat * current_quat)).c_str());
        ROS_INFO("    Inverse Error x Current Rotation: %s", to_string(Eigen::AngleAxisd(error_quat.inverse() * current_quat)).c_str());

        const double alpha = 0.2;
//        Eigen::Quaterniond target_rot = current_quat * error_quat;
//        Eigen::Quaterniond target_rot = error_quat * current_quat;
        Eigen::Quaterniond target_rot = current_quat * error_quat.inverse();
//        Eigen::Quaterniond target_rot = error_quat.inverse() * current_quat;

        const double REACHAROUND_FACTOR = 0.1;
        target_rot = current_quat.slerp(REACHAROUND_FACTOR, current_quat * error_quat.inverse());

        tf::Transform t(tf::Quaternion(target_rot.x(), target_rot.y(), target_rot.z(), target_rot.w()), tf::Vector3(target_pos.x(), target_pos.y(), target_pos.z()));
        t.getBasis().getEulerYPR(yaw, pitch, roll, 1);

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target transform
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d camera_to_target_wrist_transform = Eigen::Translation3d(target_pos) * target_rot;

        ROS_INFO("    Target Wrist Transform: %s", to_string(camera_to_target_wrist_transform).c_str());
        ROS_INFO("      Euler Angles: r: %0.3f, p: %0.3f, y: %0.3f", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        Eigen::Affine3d mount_to_target_wrist = camera_to_mount.inverse() * camera_to_target_wrist_transform;
        ROS_INFO("    Target Wrist Transform in Mount Frame: %s", to_string(mount_to_target_wrist).c_str());

        /////////////////////////////////////////
        geometry_msgs::Vector3 scale;
        scale.x = 0.10;
        scale.y = 0.01;
        scale.z = 0.01;
        visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

        for (auto& marker : triad_marker.markers) {
            marker.header.frame_id = camera_frame_;
            marker.ns = "wrist_goal";

            Eigen::Affine3d triad_marker_transform;
            tf::poseMsgToEigen(marker.pose, triad_marker_transform);
            triad_marker_transform = camera_to_target_wrist_transform * triad_marker_transform;

            tf::poseEigenToMsg(triad_marker_transform, marker.pose);
        }

        corrected_wrist_goal_pub_.publish(triad_marker);
        /////////////////////////////////////////

        // Find IK Solution that puts the end effector at the target location
        hdt::IKSolutionGenerator ikgen = robot_model_->search_all_ik_solutions(
                mount_to_target_wrist, last_joint_state_msg_->position, sbpl::utils::ToRadians(1.0));

        std::vector<double> iksol;
        bool found_ik = false;

        // loop through ik solutions until we find one that looks safe to move to
        while (ikgen(iksol)) {
            assert(iksol.size() == robot_model_->joint_names().size());

            // FOR NOW: disallow motions if the joints will not be able to reach
            // their target positions within a reasonable amount of time
            bool safe_joint_command = true;
            const double angle_threshold_degs = 45.0;
            for (std::size_t solidx = 0; solidx < iksol.size(); ++solidx) {
                double current_angle = last_joint_state_msg_->position[solidx];
                double solution_angle = iksol[solidx];
                double min_angle = robot_model_->min_limits()[solidx];
                double max_angle = robot_model_->max_limits()[solidx];

                safe_joint_command &=
                        sbpl::utils::ShortestAngleDistWithLimits(solution_angle, current_angle, min_angle, max_angle) <
                        sbpl::utils::ToRadians(angle_threshold_degs);

                if (!safe_joint_command) {
                    ROS_ERROR("Next joint state too far away from current joint state (%0.3f degs - %0.3f degs > %0.3f degs)",
                        sbpl::utils::ToDegrees(solution_angle),
                        sbpl::utils::ToDegrees(current_angle),
                        angle_threshold_degs);
                    break;
                }
            }

//            if (!safe_joint_command) {
//                ROS_WARN("Candidate joint command %s seems unsafe", to_string(msg_utils::to_degrees(iksol)).c_str());
//                continue;
//            }
//            else
            {
                found_ik = true;
                break;
            }
        }

        if (!found_ik) {
            ROS_WARN("Unable to find IK solution to move the arm towards the goal. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setAborted(result);
            continue;
        }

        // Publish the resulting command
        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.points.resize(1);
        traj_cmd.joint_names = robot_model_->joint_names();
        traj_cmd.points[0].positions = iksol;
        ROS_INFO("Publishing joint command %s", to_string(msg_utils::to_degrees(traj_cmd.points[0].positions)).c_str());
        joint_command_pub_.publish(traj_cmd);
    }

    return SUCCESS;
}

void ViservoControlExecutor::goal_callback()
{
    current_goal_ = as_->acceptNewGoal();
    ROS_WARN("Received a goal to move the wrist to %s in the camera frame", to_string(current_goal_->goal_pose).c_str());

    // TODO: make sure that the markers are in view of the camera or catch this during executive
    // for visualization:
    //     1. estimate the current wrist frame
    //     2.

    update_wrist_pose_estimate();

    geometry_msgs::Vector3 scale;
    scale.x = 0.10;
    scale.y = 0.01;
    scale.z = 0.01;
    visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

    for (auto& marker : triad_marker.markers) {
        marker.header.frame_id = camera_frame_;

        Eigen::Affine3d triad_marker_transform;
        tf::poseMsgToEigen(marker.pose, triad_marker_transform);
        triad_marker_transform = wrist_transform_estimate_ * triad_marker_transform;

        tf::poseEigenToMsg(triad_marker_transform, marker.pose);
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

    ROS_INFO("Wrist Transform Estimate = Camera-to-Marker * Marker-AttachedLink * AttachedLink-Wrist");
    ROS_INFO("    Camera-to-Marker: %s", to_string(camera_to_marker).c_str());
    ROS_INFO("    Marker-to-AttachedLink: %s", to_string(marker_to_attached_link).c_str());
    ROS_INFO("    AttachedLink-to_wrist: %s", to_string(attached_link_to_wrist).c_str());

    // combine transforms to get the camera -> wrist transform
    wrist_transform_estimate_ = camera_to_marker * marker_to_attached_link * attached_link_to_wrist;
    return true;
}

bool ViservoControlExecutor::download_marker_params()
{
    double marker_to_link_x;
    double marker_to_link_y;
    double marker_to_link_z;
    double marker_to_link_roll_degs;
    double marker_to_link_pitch_degs;
    double marker_to_link_yaw_degs;

    bool success =
            msg_utils::download_param(ph_, "tracked_marker_id", attached_marker_.marker_id) &&
            msg_utils::download_param(ph_, "tracked_marker_attached_link", attached_marker_.attached_link) &&
            msg_utils::download_param(ph_, "marker_to_link_x", marker_to_link_x) &&
            msg_utils::download_param(ph_, "marker_to_link_y", marker_to_link_y) &&
            msg_utils::download_param(ph_, "marker_to_link_z", marker_to_link_z) &&
            msg_utils::download_param(ph_, "marker_to_link_roll_deg", marker_to_link_roll_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_pitch_deg", marker_to_link_pitch_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_yaw_deg", marker_to_link_yaw_degs) &&
            msg_utils::download_param(ph_, "marker_validity_timeout", marker_validity_timeout_) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_x_m", goal_pos_tolerance_(0)) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_y_m", goal_pos_tolerance_(1)) &&
            msg_utils::download_param(ph_, "goal_pos_tolerance_z_m", goal_pos_tolerance_(2)) &&
            msg_utils::download_param(ph_, "goal_rot_tolerance_deg", goal_rot_tolerance_);

    goal_rot_tolerance_ = sbpl::utils::ToRadians(goal_rot_tolerance_);

    attached_marker_.link_to_marker = Eigen::Affine3d(
            Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX()));


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

            marker_in_msg_frame.pose = marker.pose.pose;

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
