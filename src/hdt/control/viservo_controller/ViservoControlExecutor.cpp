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
    max_translational_velocity_mps_(0.40),
    max_rotational_velocity_rps_(sbpl::utils::ToRadians(90.0)), // 90 degrees in 4 seconds
    deadband_joint_velocities_rps_(),
    minimum_joint_velocities_rps_(),
    max_joint_velocities_rps_(7, sbpl::utils::ToRadians(20.0)),
    listener_(),
    camera_frame_("asus_rgb_frame"),
    wrist_frame_("arm_7_gripper_lift_link"),
    mount_frame_("arm_mount_panel_dummy"),
    attached_marker_(),
    cmd_seqno_(0)
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

        if (last_joint_state_msg_) {
            static std::vector<int> misbehaved_joints_histogram(7, 0);
            for (std::size_t i = 0; i < last_curr_.size(); ++i) {
                if (signf(last_joint_state_msg_->position[i] - last_curr_[i], sbpl::utils::ToRadians(1.0)) !=
                    signf(last_diff_[i], sbpl::utils::ToRadians(1.0)))
                {
                    ++misbehaved_joints_histogram[i];
                }
            }
            ROS_INFO("Misbehaved joint histogram: %s", to_string(misbehaved_joints_histogram).c_str());
        }

        if (as_->isPreemptRequested()) {
            ROS_WARN("Goal preemption currently unimplemented");
        }

        hdt::ViservoCommandResult result;

        // incorporate any new marker measurements and estimate the current wrist pose based off of that and the last two joints
        if (!update_wrist_pose_estimate()) {
            ROS_WARN("Failed to update the pose of the wrist");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setSucceeded(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // publish ee estimate marker
        geometry_msgs::Vector3 scale;
        scale.x = 0.10;
        scale.y = 0.01;
        scale.z = 0.01;
        visualization_msgs::MarkerArray estimate_triad_marker = msg_utils::create_triad_marker_arr(scale);

        for (auto& marker : estimate_triad_marker.markers) {
            marker.header.frame_id = camera_frame_;
            marker.ns = "ee estimate";

            Eigen::Affine3d estimate_triad_marker;
            tf::poseMsgToEigen(marker.pose, estimate_triad_marker);
            estimate_triad_marker = wrist_transform_estimate_ * estimate_triad_marker;

            tf::poseEigenToMsg(estimate_triad_marker, marker.pose);
        }

        corrected_wrist_goal_pub_.publish(estimate_triad_marker);

        ////////////////////////////////////////////////////////////////////////////////
        // Check for termination
        ////////////////////////////////////////////////////////////////////////////////

        // 1. check whether the estimated wrist pose has reached the goal within the specified tolerance
        if (reached_goal()) {
            ROS_INFO("Wrist has reached goal. Completing action...");
            result.result = hdt::ViservoCommandResult::SUCCESS;
            as_->setSucceeded(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // 2. lost track of the marker
        if (lost_marker()) {
            ROS_WARN("Marker has been lost. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::LOST_MARKER;
            as_->setAborted(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // 3. check whether the wrist has moved too far from the goal (and the canonical path follower should retry)
        if (moved_too_far()) {
            ROS_INFO("Arm has moved too far from the goal. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::MOVED_TOO_FAR;
            as_->setAborted(result);
            stop_arm(cmd_seqno_++);
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
            stop_arm(cmd_seqno_++);
            continue;
        }

        Eigen::Affine3d camera_to_mount =
            Eigen::Translation3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()) *
            Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

        ROS_DEBUG("camera -> mount: %s", to_string(camera_to_mount).c_str());

        Eigen::Affine3d manip_to_ee;
        if (!robot_model_->compute_fk(last_joint_state_msg_->position, manip_to_ee)) {
            ROS_ERROR("Failed to compute end effector transform from joint state");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setSucceeded(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        const Eigen::Affine3d& mount_to_manip = robot_model_->mount_to_manipulator_transform();
        ROS_DEBUG("mount -> manipulator: %s", to_string(mount_to_manip).c_str());

        ROS_DEBUG("manipulator -> ee: %s", to_string(manip_to_ee).c_str());

        // camera -> ee = camera -> mount * mount -> manip * manip -> ee
        Eigen::Affine3d camera_to_ee = camera_to_mount * mount_to_manip * manip_to_ee;
        ROS_DEBUG("camera -> ee: %s", to_string(camera_to_ee).c_str());

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

        msg_utils::get_euler_ypr(camera_to_ee, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Belief [camera frame]: %s", to_string(camera_to_ee).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        msg_utils::get_euler_ypr(wrist_transform_estimate_, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Estimate [camera frame]: %s", to_string(wrist_transform_estimate_).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        msg_utils::get_euler_ypr(camera_to_mount.inverse() * wrist_transform_estimate_, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Estimate [mount frame]: %s", to_string(camera_to_mount.inverse() * wrist_transform_estimate_).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        ROS_INFO("Error: %s", to_string(error).c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target position
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Vector3d pos_error(error.translation());

        ROS_INFO("    Position Error: %s", to_string(pos_error).c_str());

#define PROLLYBAD 0
#if !PROLLYBAD
        // cap the positional error vector by the max translational velocity_mps
        if (pos_error.squaredNorm() > sqrd(max_translational_velocity_mps_)) {
            pos_error.normalize();
            pos_error *= max_translational_velocity_mps_;
        }

        ROS_INFO("    EE Velocity [camera frame]: %s", to_string(pos_error).c_str());
#endif

        Eigen::Vector3d current_wrist_pos(camera_to_ee.translation());
#if PROLLYBAD
        Eigen::Vector3d target_pos = current_wrist_pos + pos_error;
#else
        Eigen::Vector3d target_pos = current_wrist_pos + pos_error * dt;
#endif

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
#if PROLLYBAD
        double REACHAROUND_FACTOR = 1.0;
#else
        double REACHAROUND_FACTOR = angle_dist > 1e-4 ? clamp(fabs(delta_angle / angle_dist), 0.0, 1.0) : 1.0;
#endif
        Eigen::Quaterniond target_rot = current_quat.slerp(REACHAROUND_FACTOR, corrected_goal_rot);

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target transform
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d camera_to_target_wrist_transform = Eigen::Translation3d(target_pos) * target_rot;

        ROS_INFO("Target Wrist Transform [camera frame]: %s", to_string(camera_to_target_wrist_transform).c_str());
        msg_utils::get_euler_ypr(camera_to_target_wrist_transform, roll, pitch, yaw);
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::utils::ToDegrees(roll), sbpl::utils::ToDegrees(pitch), sbpl::utils::ToDegrees(yaw));

        Eigen::Affine3d mount_to_target_wrist = camera_to_mount.inverse() * camera_to_target_wrist_transform;
        ROS_INFO("Target Wrist Transform [mount frame]: %s", to_string(mount_to_target_wrist).c_str());

        // TODO: transform into the manipulator frame before IK (works for now because manip -> mount = I)

        // publish a marker for the target transform

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

        // gather a lot of candidate ik solutions
        int num_solutions_found = 0;
        while (ikgen(iksol) && num_solutions_found < max_ik_solutions) {
            ++num_solutions_found;
            ik_solutions.push_back(std::move(iksol));
        }

        if (num_solutions_found == 0) {
            ROS_ERROR("Failed to compute IK solution to move the arm towards the goal. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setAborted(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        ROS_INFO("Picking best IK solution out of %d", num_solutions_found);

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
            ROS_WARN("No IK solution to move the arm towards the goal is deemed safe. Aborting Viservo action...");
            result.result = hdt::ViservoCommandResult::STUCK;
            as_->setAborted(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // Publish the resulting command
        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.header.seq = cmd_seqno_++;
        traj_cmd.points.resize(1);
        traj_cmd.joint_names = robot_model_->joint_names();
        traj_cmd.points[0].positions = chosen_solution;

        if (is_valid_command(prev_cmd_)) {
            std::vector<double> diff;
            msg_utils::vector_diff(prev_cmd_.positions, last_joint_state_msg_->position, diff);
            msg_utils::vector_sum(traj_cmd.points[0].positions, diff, traj_cmd.points[0].positions);
        }

        std::vector<double> delta_joints;
        std::vector<double> joint_velocities;
        msg_utils::vector_diff(chosen_solution, last_joint_state_msg_->position, delta_joints);
        msg_utils::vector_mul(delta_joints, std::vector<double>(robot_model_->joint_names().size(), dt), joint_velocities);
        for (double& vel : joint_velocities) { // convert velocities to speeds
            vel = fabs(vel);
        }
        traj_cmd.points[0].velocities = joint_velocities;

        correct_joint_trajectory_cmd(*last_joint_state_msg_, prev_cmd_, traj_cmd.points[0], dt);

        ROS_INFO("Publishing joint command %s @ %s",
                to_string(msg_utils::to_degrees(traj_cmd.points[0].positions)).c_str(),
                to_string(msg_utils::to_degrees(traj_cmd.points[0].velocities)).c_str());

        last_curr_ = last_joint_state_msg_->position;
        if (msg_utils::vector_diff(traj_cmd.points[0].positions, last_curr_, last_diff_)) {
            ROS_INFO("Joint Command Difference: %s", to_string(msg_utils::to_degrees(last_diff_)).c_str());
        }
        else {
            ROS_WARN("Failed to compute difference between two vectors?");
        }
        prev_cmd_ = traj_cmd.points.front();

        // TODO: close the loop with the HDT PID controller here so that we can
        // estimate how much the wrist has moved, even if we are unable to
        // detect the AR marker. This should allow longer AR marker timeouts or
        // perhaps no timeout if the motion can be executed from only the
        // initial estimate
        joint_command_pub_.publish(traj_cmd);
    }

    return SUCCESS;
}

void ViservoControlExecutor::goal_callback()
{
    current_goal_ = as_->acceptNewGoal();
    ROS_WARN("Received a goal to move the wrist to %s in the camera frame", to_string(current_goal_->goal_pose).c_str());

    prev_cmd_.positions.clear(); // indicate that we have no previous command for this goal
    prev_cmd_.velocities.clear();

    // TODO: make sure that the markers are in view of the camera or catch this during executive
    // for visualization:
    //     1. estimate the current wrist frame
    //     2.

    // TODO: dont need to bail here upon failure to update wrist? maybe this is because only visualizations follow
    update_wrist_pose_estimate();

    geometry_msgs::Vector3 scale;
    scale.x = 0.10;
    scale.y = 0.01;
    scale.z = 0.01;
    visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

    for (auto& marker : triad_marker.markers) {
        marker.header.frame_id = camera_frame_;
        marker.ns = "initial ee estimate";

        Eigen::Affine3d triad_marker_transform;
        tf::poseMsgToEigen(marker.pose, triad_marker_transform);
        triad_marker_transform = wrist_transform_estimate_ * triad_marker_transform;

        tf::poseEigenToMsg(triad_marker_transform, marker.pose);
    }

    corrected_wrist_goal_pub_.publish(triad_marker);

    Eigen::Affine3d goal_transform;
    tf::poseMsgToEigen(current_goal_->goal_pose, goal_transform);
    Eigen::Affine3d error = msg_utils::transform_diff(goal_transform, wrist_transform_estimate_);

    // TODO: derive the corrected ee goal and publish a marker to show the goal position

    // Obtain the current transform of the ee (using joint state data) in the frame of the camera
    tf::StampedTransform transform;
    try {
        listener_.lookupTransform(camera_frame_, mount_frame_, ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Unable to lookup transform from %s to %s (%s)", mount_frame_.c_str(), camera_frame_.c_str(), ex.what());
        return;
    }

    Eigen::Affine3d camera_to_mount =
        Eigen::Translation3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()) *
        Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

    ROS_DEBUG("camera -> mount: %s", to_string(camera_to_mount).c_str());

    Eigen::Affine3d manip_to_ee;
    if (!robot_model_->compute_fk(last_joint_state_msg_->position, manip_to_ee)) {
        ROS_ERROR("Failed to compute end effector transform from joint state");
        return;
    }

    const Eigen::Affine3d& mount_to_manip = robot_model_->mount_to_manipulator_transform();
    ROS_DEBUG("mount -> manipulator: %s", to_string(mount_to_manip).c_str());

    ROS_DEBUG("manipulator -> ee: %s", to_string(manip_to_ee).c_str());

    // camera -> ee = camera -> mount * mount -> manip * manip -> ee
    Eigen::Affine3d camera_to_ee = camera_to_mount * mount_to_manip * manip_to_ee;
    ROS_DEBUG("camera -> ee: %s", to_string(camera_to_ee).c_str());

    // current/e
    Eigen::Translation3d corrected_goal_pos(Eigen::Vector3d(camera_to_ee.translation()) + Eigen::Vector3d(error.translation()));
    Eigen::Quaterniond corrected_goal_rot = Eigen::Quaterniond(camera_to_ee.rotation()) *
                                            Eigen::Quaterniond(error.rotation()).inverse();

    // TODO: AAAAAH MASSIVE COPYPASTA HERE AND ABOVE
    Eigen::Affine3d corrected_ee_goal_transform = corrected_goal_pos * corrected_goal_rot;

    triad_marker = msg_utils::create_triad_marker_arr(scale); // refresh marker transforms
    for (auto& marker : triad_marker.markers) {
        marker.header.frame_id = camera_frame_;
        marker.ns = "corrected_ee_goal";

        Eigen::Affine3d triad_marker_transform;
        tf::poseMsgToEigen(marker.pose, triad_marker_transform);
        triad_marker_transform = corrected_ee_goal_transform * triad_marker_transform;

        tf::poseEigenToMsg(triad_marker_transform, marker.pose);
    }

    corrected_wrist_goal_pub_.publish(triad_marker);

    // make sure that we're able to come up with an IK solution for the target goal ee pose

    // TODO: to manipulator frame
    Eigen::Affine3d mount_to_corrected_goal = camera_to_mount.inverse() * corrected_goal_rot;
    auto ikgen = robot_model_->search_all_ik_solutions(mount_to_corrected_goal, last_joint_state_msg_->position, sbpl::utils::ToRadians(1.0));
    std::vector<double> sol;
    if (!ikgen(sol)) {
        ROS_ERROR("No IK Solution exists for corrected goal position");
    }
    else {
        ROS_INFO("Found valid IK solution for corrected goal pose at %s", to_string(sol).c_str());
    }
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

    bool found = false;
    ros::Time marker_stamp;
    for (const auto& marker : last_ar_markers_msg_->markers) {
        if (marker.id == attached_marker_.marker_id) {
            found = true;
            marker_stamp = marker.header.stamp;
            break;
        }
    }

    if (!found) {
        ROS_WARN("Did not find AR Marker in AR Markers message");
        return true;
    }

    if (ros::Time::now() > marker_stamp + ros::Duration(marker_validity_timeout_)) {
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

    deadband_joint_velocities_rps_.resize(7);
    minimum_joint_velocities_rps_.resize(7);

    std::vector<double> deadband_joint_velocities_dps;
    std::vector<double> minimum_joint_velocities_dps;
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
            msg_utils::download_param(ph_, "goal_rot_tolerance_deg", goal_rot_tolerance_) &&
            msg_utils::download_param(ph_, "deadband_joint_velocities_deg", deadband_joint_velocities_dps) &&
            msg_utils::download_param(ph_, "minimum_joint_velocities_deg", minimum_joint_velocities_dps)
    ; // LO'D!

    success &= deadband_joint_velocities_dps.size() == 7 && minimum_joint_velocities_dps.size() == 7;
    if (!success) {
        return false;
    }

    // convert to radians
    deadband_joint_velocities_rps_ = msg_utils::to_radians(deadband_joint_velocities_dps);
    minimum_joint_velocities_rps_ = msg_utils::to_radians(minimum_joint_velocities_dps);

    goal_rot_tolerance_ = sbpl::utils::ToRadians(goal_rot_tolerance_);

    attached_marker_.link_to_marker = Eigen::Affine3d(
            Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

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

            const std::string& msg_frame = marker.header.frame_id;
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
                    ROS_ERROR("Failed to transform from frame '%s' to '%s' (%s)", marker_in_msg_frame.header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
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

void ViservoControlExecutor::correct_joint_trajectory_cmd(
    const sensor_msgs::JointState& from,
    const trajectory_msgs::JointTrajectoryPoint& prev_cmd,
    trajectory_msgs::JointTrajectoryPoint& curr_cmd,
    double dt)
{
    std::vector<double> diff;
    msg_utils::vector_diff(curr_cmd.positions, from.position, diff);

    for (std::size_t i = 0; i < from.position.size(); ++i) {
        if (fabs(diff[i]) < deadband_joint_velocities_rps_[i] * dt) {
            curr_cmd.positions[i] = from.position[i]; //last_to.empty() ? from.position[i] : last_to[i]; //from.position[i];
        }
        else if (fabs(diff[i]) < minimum_joint_velocities_rps_[i] * dt) {
            curr_cmd.positions[i] = from.position[i] + signf(diff[i]) * minimum_joint_velocities_rps_[i];
        }
    }

    // apply velocity corrections
    for (std::size_t i = 0; i < from.position.size(); ++i) {
        if (curr_cmd.velocities[i] < deadband_joint_velocities_rps_[i] * dt) {
            curr_cmd.velocities[i] = 0.0;
        }
        else if (fabs(curr_cmd.velocities[i]) < minimum_joint_velocities_rps_[i] * dt) {
            curr_cmd.velocities[i] = minimum_joint_velocities_rps_[i];
        }
    }
}

bool ViservoControlExecutor::is_valid_command(const trajectory_msgs::JointTrajectoryPoint& cmd)
{
    return cmd.positions.size() == robot_model_->joint_names().size() &&
           cmd.velocities.size() == robot_model_->joint_names().size();
}

void ViservoControlExecutor::stop_arm(int seqno)
{
    trajectory_msgs::JointTrajectory traj_cmd;
    traj_cmd.header.frame_id = "";
    traj_cmd.header.seq = seqno;
    traj_cmd.header.stamp = ros::Time::now();
    traj_cmd.joint_names = robot_model_->joint_names();
    traj_cmd.points.resize(1);
    traj_cmd.points[0].positions = last_joint_state_msg_->position;
    traj_cmd.points[0].velocities = std::vector<double>(robot_model_->joint_names().size(), 0.0);

    joint_command_pub_.publish(traj_cmd);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viservo_control_executor");
    return ViservoControlExecutor().run();
}
