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
    max_rotational_velocity_rps_(sbpl::utils::ToRadians(0.25 * 90.0)), // 90 degrees in 4 seconds
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
    corrected_wrist_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ViservoControlExecutor::joint_states_cb, this);
    ar_marker_sub_ = nh_.subscribe("/ar_pose_marker", 1, &ViservoControlExecutor::ar_markers_cb, this);

    ROS_INFO("Starting action server 'viservo_command'...");
    as_->start();
    ROS_INFO("Action server started");

    // TODO: Procedure for setting up and configuring markers for visual servoing
    //     0. Prepare and measure markers.
    //     1. Attach an AR marker to each side of the gripper.
    //     2. Measure the offset from each AR marker to the wrist link frame, when the gripper is fully opened.
    //     3. Before visual servoing, assert that the gripper is fully opened so that the estimated wrist position is accurate.

    // Main executive loop overview:
    //
    //     1. Receive a goal pose in the frame of the camera for the tracked marker bundle to achieve.
    //         This is likely the same wrist goal that is given to the planner for the vanilla trajectory follower to attempt to achieve.
    //     2. Estimate the actual pose of the wrist by incorporating the most recent measurements of the tracked AR markers.
    //     translate this pose into the wrist frame of the arm
    //     apply P control to determine the direction that the wrist should travel
    //     apply the jacobian in the direction of the error to derive a joint state that moves the arm toward the goal
    //     send that joint state to the arm controller

    const double rate_hz = 10.0;
    ros::Rate executive_rate(rate_hz);
    double dt = 1.0 / rate_hz;
    while (ros::ok())
    {
        // ensure that we always execute this loop no faster than 10 Hz.
        RunUponDestruction rod([&]() { executive_rate.sleep(); });

        ros::spinOnce();

        if (!as_->isActive()) {
            continue;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Proceed with current goal
        ////////////////////////////////////////////////////////////////////////////////

        ROS_WARN("Executing viservo control. Current joint state is %s",
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

        ////////////////////////////////////////////////////////////////////////////////
        // Check for termination
        ////////////////////////////////////////////////////////////////////////////////

        hdt::ViservoCommandResult result;

        // 1. check whether the estimated wrist pose has reached the goal within the specified tolerance
        if (reached_goal()) {
            ROS_INFO("Wrist has reached goal. Completing action...");
            result.result = hdt::ViservoCommandResult::SUCCESS;
            as_->setSucceeded(result);
            continue;
        }

        // 2. lost track of the marker
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

        ////////////////////////////////////////////////////////////////////////////////
        // Compute the current ee transform from joint state data
        ////////////////////////////////////////////////////////////////////////////////

        // Obtain the current transform of the ee (using joint state data) in the frame of the camera

        tf::StampedTransform transform;
        try {
            listener_.lookupTransform(camera_frame_, mount_frame_, ros::Time(0), transform);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("Unable to lookup transform from %s to %s (%s)", mount_frame_.c_str(), camera_frame_.c_str(), ex.what());
            continue;
        }

        Eigen::Affine3d camera_to_mount =
            Eigen::Translation3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()) *
            Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

        ROS_INFO("camera -> mount: %s", to_string(camera_to_mount).c_str());

        Eigen::Affine3d manip_to_ee;
        if (!robot_model_->compute_fk(last_joint_state_msg_->position, manip_to_ee)) {
            ROS_ERROR("Failed to compute end effector transform from joint state");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setSucceeded(result);
            continue;
        }

        const Eigen::Affine3d& mount_to_manip = robot_model_->mount_to_manipulator_transform();
        ROS_INFO("mount -> manipulator: %s", to_string(mount_to_manip).c_str());

        ROS_INFO("manipulator -> ee: %s", to_string(manip_to_ee).c_str());

        // camera -> ee = camera -> mount * mount -> manip * manip -> ee
        Eigen::Affine3d camera_to_ee = camera_to_mount * mount_to_manip * manip_to_ee;
        ROS_INFO("camera -> ee: %s", to_string(camera_to_ee).c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // Compute the error between the goal transform and the ee estimate
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d goal_wrist_transform; // in camera frame
        tf::poseMsgToEigen(current_goal_->goal_pose, goal_wrist_transform);

        // pretty print euler angles
        double roll, pitch, yaw;

        // transform from the goal to the current pose
        Eigen::Affine3d error = msg_utils::transform_diff(goal_wrist_transform, wrist_transform_estimate_);

        msg_utils::get_euler_ypr(goal_wrist_transform, yaw, pitch, roll);
        ROS_INFO("Goal Wrist Transform [camera frame]: %s", to_string(goal_wrist_transform).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        msg_utils::get_euler_ypr(wrist_transform_estimate_, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform [camera frame]: %s", to_string(wrist_transform_estimate_).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));
        ROS_INFO("Error: %s", to_string(error).c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target position
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Vector3d pos_error(error.translation());

        ROS_INFO("    Position Error: %s", to_string(pos_error).c_str());

        // cap the positional error vector by the max translational velocity_mps
        if (pos_error.norm() > max_translational_velocity_mps_ * max_translational_velocity_mps_) {
            pos_error.normalize();
            pos_error *= max_translational_velocity_mps_;
        }

        Eigen::Vector3d current_wrist_pos(camera_to_ee.translation());
        Eigen::Vector3d target_pos = current_wrist_pos + pos_error * dt;

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target rotation
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::AngleAxisd aa_error(error.rotation());
        double angle_error = aa_error.angle();

        ROS_INFO("    Rotation Error: %0.3f degs about %s", sbpl::utils::ToDegrees(angle_error), to_string(aa_error.axis()).c_str());

        double angular_velocity_rps = angle_error;
        if (fabs(angular_velocity_rps) > fabs(max_rotational_velocity_rps_)) {
            angular_velocity_rps = clamp(angular_velocity_rps, -max_rotational_velocity_rps_, max_rotational_velocity_rps_);
        }

        Eigen::Quaterniond error_quat(error.rotation());
        Eigen::Quaterniond current_quat(camera_to_ee.rotation());

//        Eigen::Quaterniond corrected_goal_rot = current_quat * error_quat;
//        Eigen::Quaterniond corrected_goal_rot = error_quat * current_quat;
        Eigen::Quaterniond corrected_goal_rot = current_quat * error_quat.inverse();
//        Eigen::Quaterniond corrected_goal_rot = error_quat.inverse() * current_quat;

        double delta_angle = angular_velocity_rps * dt;
        double angle_dist = current_quat.angularDistance(corrected_goal_rot);
        double REACHAROUND_FACTOR = angle_dist > 1e-4 ? clamp(fabs(delta_angle / angle_dist), 0.0, 1.0) : 1.0;
        Eigen::Quaterniond target_rot = current_quat.slerp(REACHAROUND_FACTOR, corrected_goal_rot);

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target transform
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d camera_to_target_wrist_transform = Eigen::Translation3d(target_pos) * target_rot;

        ROS_INFO("Target Wrist Transform: %s", to_string(camera_to_target_wrist_transform).c_str());
        msg_utils::get_euler_ypr(camera_to_target_wrist_transform, roll, pitch, yaw);
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        Eigen::Affine3d mount_to_target_wrist = camera_to_mount.inverse() * camera_to_target_wrist_transform;
        ROS_INFO("Target Wrist Transform [mount frame]: %s", to_string(mount_to_target_wrist).c_str());

        // publish a marker for the target transform

        geometry_msgs::Vector3 scale;
        scale.x = 0.10;
        scale.y = 0.01;
        scale.z = 0.01;
        visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

        for (auto& marker : triad_marker.markers) {
            marker.header.frame_id = camera_frame_;
            marker.ns = "viservo_wrist_target";

            Eigen::Affine3d triad_marker_transform;
            tf::poseMsgToEigen(marker.pose, triad_marker_transform);
            triad_marker_transform = camera_to_target_wrist_transform * triad_marker_transform;

            tf::poseEigenToMsg(triad_marker_transform, marker.pose);
        }

        corrected_wrist_goal_pub_.publish(triad_marker);

        ////////////////////////////////////////////////////////////////////////////////
        // Find a reasonable IK solution that puts the end effector at the target location
        ////////////////////////////////////////////////////////////////////////////////

        hdt::IKSolutionGenerator ikgen = robot_model_->search_all_ik_solutions(
                mount_to_target_wrist, last_joint_state_msg_->position, sbpl::utils::ToRadians(1.0));


        std::vector<std::vector<double>> ik_solutions;
        static const int max_ik_solutions = 100;
        std::vector<double> iksol;

        int num_solutions_found = 0;
        while (ikgen(iksol) && num_solutions_found < max_ik_solutions) {
            ++num_solutions_found;
            ik_solutions.push_back(std::move(iksol));
        }

        bool found_ik = false;
        double best_dist = -1.0;
        std::vector<double> chosen_solution;
        for (const std::vector<double>& sol : ik_solutions) {
            if (safe_joint_delta(last_joint_state_msg_->position, sol))
            {
                if (!found_ik) {
                    found_ik = true;
                    best_dist = hdt::ComputeJointStateL2NormSqrd(last_joint_state_msg_->position, sol);
                    chosen_solution = sol;
                }
                else {
                    double dist = hdt::ComputeJointStateL2NormSqrd(last_joint_state_msg_->position, sol);
                    if (dist < best_dist) {
                        best_dist = dist;
                        chosen_solution = sol;
                    }
                }
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
        traj_cmd.points[0].positions = chosen_solution;
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
        marker.ns = "corrected_ee_goal";

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
    if (!last_joint_state_msg_) {
        ROS_WARN("Have yet to receive a valid Joint State message");
        return false;
    }

    if (!last_ar_markers_msg_) {
        ROS_WARN("Have yet to receive valid AR markers message");
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

bool ViservoControlExecutor::safe_joint_delta(const std::vector<double>& from, const std::vector<double>& to) const
{
    assert(from.size() == to.size() && from.size() == 7);

    const double angle_threshold_degs = 45.0;
    for (std::size_t solidx = 0; solidx < from.size(); ++solidx) {
        double from_angle = from[solidx];
        double to_angle = to[solidx];
        double min_angle = robot_model_->min_limits()[solidx];
        double max_angle = robot_model_->max_limits()[solidx];

        bool safe_joint_command =
                sbpl::utils::ShortestAngleDistWithLimits(to_angle, from_angle, min_angle, max_angle) <
                sbpl::utils::ToRadians(angle_threshold_degs);

        if (!safe_joint_command) {
            ROS_ERROR("Next joint state too far away from current joint state (%0.3f degs - %0.3f degs > %0.3f degs)",
                sbpl::utils::ToDegrees(to_angle),
                sbpl::utils::ToDegrees(from_angle),
                angle_threshold_degs);
            return false;
        }
    }

    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viservo_control_executor");
    return ViservoControlExecutor().run();
}
