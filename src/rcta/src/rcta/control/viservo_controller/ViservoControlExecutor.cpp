#include "ViservoControlExecutor.h"

// standard includes
#include <cassert>
#include <cmath>
#include <sstream>

// system includes
#include <boost/date_time.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/angles.h>
#include <smpl/angles.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <spellbook/utils/utils.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/Marker.h>

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<const urdf::Link> LinkConstPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<const urdf::Joint> JointConstPtr;

std::string to_string(const KDL::Vector& v)
{
    std::stringstream ss;
    ss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return ss.str();
}

std::string to_string(const KDL::Rotation& r)
{
    double w, x, y, z;
    r.GetQuaternion(x, y, z, w);

    std::stringstream ss;
    ss << "(" << w << ", " << x << ", " << y << ", " << z << ")";
    return ss.str();
}

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
    curr_joint_state_(),
    last_ar_markers_msg_(),
    marker_validity_timeout_(), // read from param server
    wrist_transform_estimate_(),
    max_translational_velocity_mps_(0.40),
    max_rotational_velocity_rps_(sbpl::angles::to_radians(90.0)), // 90 degrees in 4 seconds
    deadband_joint_velocities_rps_(),
    minimum_joint_velocities_rps_(),
    max_joint_velocities_rps_(7, sbpl::angles::to_radians(20.0)),
    listener_(),
    camera_frame_("camera_rgb_frame"),
    wrist_frame_("arm_7_gripper_lift_link"),
    mount_frame_("arm_mount_panel_dummy"),
    attached_marker_(),
    cmd_seqno_(0),
    misbehaved_joints_histogram_(7, 0),
    kdl_chain_(),
    fv_solver_()
{
}

bool ViservoControlExecutor::initialize()
{
    as_.reset(new ViservoCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Viservo Command Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&ViservoControlExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&ViservoControlExecutor::preempt_callback, this));

    if (!download_marker_params()) {
        return false;
    }

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to instantiate Robot Model");
        return false;
    }

    const std::string& chain_root_link = "arm_mount_panel_dummy";
    const std::string& chain_tip_link = robot_model_->joint_names().back() + "_link";
    const int free_angle = 4;
    kdl_robot_model_.reset(new sbpl::motion::KDLRobotModel(chain_root_link, chain_tip_link, free_angle));
    if (!kdl_robot_model_) {
        ROS_ERROR("Failed to instantiate KDL Robot Model");
    }

    if (!kdl_robot_model_->init(urdf_string, robot_model_->joint_names())) {
        ROS_ERROR("Failed to initialize KDL Robot Model");
        return false;
    }

    // BECAUSE I HAVEN'T INSTANTIATED ENOUGH ROBOT MODELS YET, OK?!
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(urdf_string);
    if (!urdf_model) {
        ROS_WARN("Failed to parse URDF from XML string");
        return false;
    }

    const std::string arm_link_1_name = "arm_1_shoulder_twist_link";
    const std::string gripper_base_link_name = "arm_7_gripper_lift_link";

    LinkConstPtr first_arm_link = urdf_model->getLink(arm_link_1_name);
    LinkConstPtr mount_link = first_arm_link->getParent();
    LinkConstPtr gripper_link = urdf_model->getLink(gripper_base_link_name);

    if (!first_arm_link || !mount_link || !gripper_link) {
        ROS_ERROR("Some required link doesn't exist");
        return false;
    }

    ROS_INFO("Found first_arm_link @ %p", first_arm_link.get());
    ROS_INFO("Found mount_link @ %p", mount_link.get());
    ROS_INFO("Found gripper_link @ %p", gripper_link.get());

    // Build the kinematic chain and forward velocity solver
    LinkConstPtr curr_link = first_arm_link;
    while (curr_link != gripper_link) {
        ROS_INFO("Current Link: %s", curr_link->name.c_str());
        JointConstPtr parent_joint = curr_link->parent_joint;

        JointConstPtr child_joint = curr_link->child_joints.front(); // Assume One Joint
        LinkConstPtr child_link = urdf_model->getLink(child_joint->child_link_name);

        KDL::Vector joint_origin(0, 0, 0);
        KDL::Vector joint_axis(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);

        // find the transform to the tip of the segment
        double ax = 0, ay = 0, az = 0, qx = 0, qy = 0, qz = 0, qw = 1;
        if (child_link != gripper_link) {
            ax = child_joint->parent_to_joint_origin_transform.position.x;
            ay = child_joint->parent_to_joint_origin_transform.position.y;
            az = child_joint->parent_to_joint_origin_transform.position.z;
            qx = child_joint->parent_to_joint_origin_transform.rotation.x;
            qy = child_joint->parent_to_joint_origin_transform.rotation.y;
            qz = child_joint->parent_to_joint_origin_transform.rotation.z;
            qw = child_joint->parent_to_joint_origin_transform.rotation.w;
        }

        KDL::Vector tip_position(ax, ay, az);
        KDL::Rotation tip_rotation(KDL::Rotation::Quaternion(qx, qy, qz, qw));

        kdl_chain_.addSegment(KDL::Segment(
                KDL::Joint(joint_origin, joint_axis, KDL::Joint::RotAxis),
                KDL::Frame(tip_rotation, tip_position)));

        ROS_INFO("Joint: %s", parent_joint->name.c_str());
        ROS_INFO("    Joint Origin: (%0.3f, %0.3f, %0.3f)", joint_origin(0), joint_origin(1), joint_origin(2));
        ROS_INFO("    Joint Axis: (%0.3f, %0.3f, %0.3f)", joint_axis(0), joint_axis(1), joint_axis(2));
        ROS_INFO("    Tip Position: (%0.3f, %0.3f, %0.3f)", tip_position(0), tip_position(1), tip_position(2));
        ROS_INFO("    Tip Rotation: (%0.3f, %0.3f, %0.3f, %0.3f)", qw, qx, qy, qz);

        curr_link = child_link;
    }

    ROS_INFO("Constructed KDL Chain with %u joints", kdl_chain_.getNrOfJoints());

    fv_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    if (!fv_solver_) {
        ROS_ERROR("Failed to instantiate KDL Chain Fk Solver Vel recursive");
        return false;
    }

    joint_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    corrected_wrist_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ViservoControlExecutor::joint_states_cb, this);
    ar_marker_sub_ = nh_.subscribe("/ar_pose_marker", 1, &ViservoControlExecutor::ar_markers_cb, this);

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

int ViservoControlExecutor::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

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
    while (ros::ok()) {
        // ensure that we always execute this loop no faster than 10 Hz.
        RunUponDestruction rod([&]() { executive_rate.sleep(); });

        ros::spinOnce();

        if (!as_->isActive()) {
            continue;
        }

        if (as_->isPreemptRequested()) {
            ROS_WARN("Goal preemption currently unimplemented");
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Proceed with current goal
        ////////////////////////////////////////////////////////////////////////////////

        if (!curr_joint_state_) {
            ROS_WARN("Goal is active but not joint state has been received");
            continue;
        }

        ROS_WARN("Executing viservo control. Current joint state is %s", to_string(msg_utils::to_degrees(curr_joint_state_->position)).c_str());

        update_histogram();

        rcta::ViservoCommandResult result;

        ////////////////////////////////////////////////////////////////////////////////
        // Incorporate latest measurements from joint states and AR markers
        ////////////////////////////////////////////////////////////////////////////////

        // Retrieve (static) transform from camera to mount using tf
        Eigen::Affine3d camera_to_mount;
        if (!lookup_transform(camera_frame_, mount_frame_, ros::Time(0), camera_to_mount)) {
            stop_arm(cmd_seqno_++);
            continue;
        }

        ROS_DEBUG("camera -> mount: %s", to_string(camera_to_mount).c_str());

        // Obtain the current transform of the ee (using joint state data) in the frame of the camera
        // Note: this could probably be done with another simple call to tf but for some reason I hate tf
        Eigen::Affine3d manip_to_ee;
        if (!robot_model_->compute_fk(curr_joint_state_->position, manip_to_ee)) {
            std::string error = "Failed to compute end effector transform from joint state";
            ROS_ERROR("%s", error.c_str());
            result.result = rcta::ViservoCommandResult::STUCK;
            as_->setAborted(result, error);
            stop_arm(cmd_seqno_++);
            continue;
        }

        const Eigen::Affine3d& mount_to_manip = robot_model_->mount_to_manipulator_transform();
        ROS_DEBUG("mount -> manipulator: %s", to_string(mount_to_manip).c_str());

        ROS_DEBUG("manipulator -> ee: %s", to_string(manip_to_ee).c_str());

        // camera -> ee = camera -> mount * mount -> manip * manip -> ee
        Eigen::Affine3d camera_to_ee = camera_to_mount * mount_to_manip * manip_to_ee;
        ROS_DEBUG("camera -> ee: %s", to_string(camera_to_ee).c_str());

        // Compute End Effector velocity in the camera frame
        KDL::FrameVel fv_frame = compute_ee_velocity(*curr_joint_state_);

        ROS_INFO("EE at %s", to_string(fv_frame.p.p).c_str());
        ROS_INFO("Rotating %0.3f deg/s about %s", sbpl::angles::to_degrees(fv_frame.M.w.Norm()), to_string(fv_frame.M.w).c_str());
        ROS_INFO("EE moving at %s (%0.3f m/s)", to_string(fv_frame.p.v).c_str(), fv_frame.p.v.Norm());

        Eigen::Vector3d ee_vel_mount_frame(fv_frame.p.v(0), fv_frame.p.v(1), fv_frame.p.v(2));
        Eigen::Vector3d ee_vel_camera_frame = camera_to_mount.rotation() * ee_vel_mount_frame;

        Eigen::Vector3d ee_rot_vel_mount_frame(fv_frame.M.w(0), fv_frame.M.w(2), fv_frame.M.w(2));
        Eigen::Vector3d ee_rot_vel_camera_frame = camera_to_mount.rotation() * ee_rot_vel_mount_frame;

        ROS_INFO("EE Pos Velocity [camera frame]: %s", to_string(ee_vel_camera_frame).c_str());
        ROS_INFO("EE Rot Velocity [camera frame]: %0.3f deg/s about %s", sbpl::angles::to_degrees(ee_rot_vel_camera_frame.norm()), to_string(ee_rot_vel_camera_frame.normalized()).c_str());

        // Incorporate new marker measurements and estimate the current wrist pose from them
        if (!update_wrist_pose_estimate()) {
        	ros::Time now = ros::Time::now();
        	const ros::Duration initial_marker_estimate_timeout(5.0);
        	if (now > goal_start_time_ + initial_marker_estimate_timeout) {
        		ROS_ERROR("Have not seen the AR marker and it's been %0.3f seconds since we received the goal", initial_marker_estimate_timeout.toSec());
        		result.result = rcta::ViservoCommandResult::LOST_MARKER;
        		as_->setAborted(result);
        		stop_arm(cmd_seqno_++);
        		continue;
        	}
        	else {
        		ROS_WARN("Failed to update the pose of the wrist");
        		continue;
        	}
        }

        publish_triad_marker("ee estimate", wrist_transform_estimate_, camera_frame_);

        ////////////////////////////////////////////////////////////////////////////////
        // Check for termination
        ////////////////////////////////////////////////////////////////////////////////

        // 1. check whether the estimated wrist pose has reached the goal within the specified tolerance
        if (reached_goal()) {
            ROS_INFO("Wrist has reached goal. Completing action...");
            result.result = rcta::ViservoCommandResult::SUCCESS;
            as_->setSucceeded(result);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // 2. lost track of the marker
        if (lost_marker()) {
            std::string error = "Marker has been lost. Aborting Viservo action...";
            ROS_WARN("%s", error.c_str());
            result.result = rcta::ViservoCommandResult::LOST_MARKER;
            as_->setAborted(result, error);
            stop_arm(cmd_seqno_++);
            continue;
        }

        // 3. check whether the wrist has moved too far from the goal (and the canonical path follower should retry)
        if (moved_too_far()) {
            std::string error = "Arm has moved too far from the goal. Aborting Viservo action...";
            ROS_INFO("%s", error.c_str());
            result.result = rcta::ViservoCommandResult::MOVED_TOO_FAR;
            as_->setAborted(result, error);
            stop_arm(cmd_seqno_++);
            continue;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target ee transform
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d goal_wrist_transform; // in camera frame
        tf::poseMsgToEigen(current_goal_->goal_pose, goal_wrist_transform);

        // pretty print euler angles
        double roll, pitch, yaw;

        // transform from the goal to the current pose
        Eigen::Affine3d error = msg_utils::transform_diff(goal_wrist_transform, wrist_transform_estimate_);

        msg_utils::get_euler_ypr(goal_wrist_transform, yaw, pitch, roll);
        ROS_INFO("Goal Wrist Transform [camera frame]: %s", to_string(goal_wrist_transform).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::angles::to_degrees(roll), sbpl::angles::to_degrees(pitch), sbpl::angles::to_degrees(yaw));

        msg_utils::get_euler_ypr(camera_to_ee, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Belief [camera frame]: %s", to_string(camera_to_ee).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::angles::to_degrees(roll), sbpl::angles::to_degrees(pitch), sbpl::angles::to_degrees(yaw));

        msg_utils::get_euler_ypr(wrist_transform_estimate_, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Estimate [camera frame]: %s", to_string(wrist_transform_estimate_).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::angles::to_degrees(roll), sbpl::angles::to_degrees(pitch), sbpl::angles::to_degrees(yaw));

        msg_utils::get_euler_ypr(camera_to_mount.inverse() * wrist_transform_estimate_, yaw, pitch, roll);
        ROS_INFO("Curr Wrist Transform Estimate [mount frame]: %s", to_string(camera_to_mount.inverse() * wrist_transform_estimate_).c_str());
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::angles::to_degrees(roll), sbpl::angles::to_degrees(pitch), sbpl::angles::to_degrees(yaw));

        ROS_INFO("Error: %s", to_string(error).c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // Compute target position
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Vector3d pos_error(error.translation());

        ROS_INFO("    Position Error: %s", to_string(pos_error).c_str());

#define PROLLYBAD 0
#if !PROLLYBAD
        // the desired velocity; cap the positional error vector by the max translational velocity_mps
        if (pos_error.squaredNorm() > sqrd(max_translational_velocity_mps_)) {
            pos_error.normalize();
            pos_error *= max_translational_velocity_mps_;
        }

//        accum_ee_vel_error += (pos_error - ee_vel_camera_frame);
//        accum_ee_vel_error *= 0.95;
//        pos_error = KP_ * (pos_error - ee_vel_camera_frame) + KI_ * accum_ee_vel_error; // P TO THE DESIRED FOO!!!!!!
//
//        ROS_INFO("    Accumulated EE Velocity Error [camera frame]: %s", to_string(accum_ee_vel_error).c_str());
//        ROS_INFO("    EE Velocity [camera frame]: %s", to_string(pos_error).c_str());
//
//        // the desired velocity; cap the positional error vector by the max translational velocity_mps
//        if (pos_error.squaredNorm() > sqrd(max_translational_velocity_mps_)) {
//            pos_error.normalize();
//            pos_error *= max_translational_velocity_mps_;
//        }
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

        ROS_INFO("    Rotation Error: %0.3f degs about %s", sbpl::angles::to_degrees(angle_error), to_string(aa_error.axis()).c_str());

        double angular_velocity_rps = angle_error;
        if (fabs(angular_velocity_rps) > fabs(max_rotational_velocity_rps_)) {
            angular_velocity_rps =
                    clamp(angular_velocity_rps, -max_rotational_velocity_rps_, max_rotational_velocity_rps_);
        }

        double real_angular_velocity_rps = ee_rot_vel_camera_frame.norm();

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
        // Construct target transform
        ////////////////////////////////////////////////////////////////////////////////

        Eigen::Affine3d camera_to_target_wrist_transform = Eigen::Translation3d(target_pos) * target_rot;

        ROS_INFO("Target Wrist Transform [camera frame]: %s", to_string(camera_to_target_wrist_transform).c_str());
        msg_utils::get_euler_ypr(camera_to_target_wrist_transform, roll, pitch, yaw);
        ROS_INFO("    Euler Angles: r: %0.3f degs, p: %0.3f degs, y: %0.3f degs", sbpl::angles::to_degrees(roll), sbpl::angles::to_degrees(pitch), sbpl::angles::to_degrees(yaw));

        Eigen::Affine3d mount_to_target_wrist = camera_to_mount.inverse() * camera_to_target_wrist_transform;
        ROS_INFO("Target Wrist Transform [mount frame]: %s", to_string(mount_to_target_wrist).c_str());

        publish_triad_marker("viservo_wrist_target", camera_to_target_wrist_transform, camera_frame_);

        ////////////////////////////////////////////////////////////////////////////////
        // Find a reasonable IK solution that puts the end effector at the target location
        ////////////////////////////////////////////////////////////////////////////////

        std::vector<double> chosen_solution;
        std::string why;
        if (!choose_best_ik_solution(mount_to_target_wrist, curr_joint_state_->position, chosen_solution, why)) {
            std::stringstream ss;
            ss << "Failed to find target joint state (" << why << "). Aborting Viservo action...";
            ROS_WARN("%s", ss.str().c_str());
            result.result = rcta::ViservoCommandResult::STUCK;
            as_->setAborted(result, ss.str());
            stop_arm(cmd_seqno_++);
            continue;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Publish the resulting command
        ////////////////////////////////////////////////////////////////////////////////

        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.header.seq = cmd_seqno_++;
        traj_cmd.points.resize(1);
        traj_cmd.joint_names = robot_model_->joint_names();
        traj_cmd.points[0].positions = chosen_solution;

        // SUBTRACT BACK OUT THE ERROR BETWEEN NOW AND THE PREVIOUS COMMAND
        if (is_valid_command(prev_cmd_)) {
            std::vector<double> diff;
            msg_utils::vector_diff(prev_cmd_.positions, curr_joint_state_->position, diff);
            msg_utils::vector_sum(traj_cmd.points[0].positions, diff, traj_cmd.points[0].positions);
        }

        // clamp the target joint position to be within joint limits
        for (size_t i = 0; i < robot_model_->joint_names().size(); ++i)
            traj_cmd.points[0].positions[i] =
                    clamp(traj_cmd.points[0].positions[i], robot_model_->min_limits()[i], robot_model_->max_limits()[i]);

        std::vector<double> delta_joints;
        std::vector<double> joint_velocities;
        msg_utils::vector_diff(chosen_solution, curr_joint_state_->position, delta_joints);
        msg_utils::vector_mul(
                delta_joints, std::vector<double>(robot_model_->joint_names().size(), 1.0 / dt), joint_velocities);
        for (double& vel : joint_velocities) { // convert velocities to speeds
            vel = fabs(vel);
        }
        traj_cmd.points[0].velocities = joint_velocities;

//        correct_joint_trajectory_cmd(*curr_joint_state_, prev_cmd_, traj_cmd.points[0], dt);

        ROS_INFO("Publishing joint command %s @ %s",
                to_string(msg_utils::to_degrees(traj_cmd.points[0].positions)).c_str(),
                to_string(msg_utils::to_degrees(traj_cmd.points[0].velocities)).c_str());

        last_curr_ = curr_joint_state_->position;
        bool res = msg_utils::vector_diff(traj_cmd.points[0].positions, last_curr_, last_diff_);
        assert(res);
        ROS_INFO("Joint Command Difference: %s", to_string(msg_utils::to_degrees(last_diff_)).c_str());

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
    goal_start_time_ = ros::Time::now();
    ROS_WARN("Received a goal to move the wrist to %s in the camera frame", to_string(current_goal_->goal_pose).c_str());

    prev_cmd_.positions.clear(); // indicate that we have no previous command for this goal
    prev_cmd_.velocities.clear();
    accum_ee_vel_error = Eigen::Vector3d(0, 0, 0);

    // TODO: make sure that the markers are in view of the camera or catch this during executive
    // for visualization:
    //     1. estimate the current wrist frame
    //     2.

    // TODO: dont need to bail here upon failure to update wrist? maybe this is because only visualizations follow
    update_wrist_pose_estimate();

    publish_triad_marker("initial_ee_estimate", wrist_transform_estimate_, camera_frame_);

    Eigen::Affine3d goal_transform;
    tf::poseMsgToEigen(current_goal_->goal_pose, goal_transform);
    Eigen::Affine3d error = msg_utils::transform_diff(goal_transform, wrist_transform_estimate_);

    // TODO: derive the corrected ee goal and publish a marker to show the goal position

    // Obtain the current transform of the ee (using joint state data) in the frame of the camera
    Eigen::Affine3d camera_to_mount;
    if (!lookup_transform(camera_frame_, mount_frame_, ros::Time(0), camera_to_mount)) {
        return;
    }

    ROS_DEBUG("camera -> mount: %s", to_string(camera_to_mount).c_str());

    Eigen::Affine3d manip_to_ee;
    if (!robot_model_->compute_fk(curr_joint_state_->position, manip_to_ee)) {
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

    Eigen::Affine3d corrected_ee_goal_transform = corrected_goal_pos * corrected_goal_rot;

    publish_triad_marker("corrected_ee_goal", corrected_ee_goal_transform, camera_frame_);

    // make sure that we're able to come up with an IK solution for the target goal ee pose

    // TODO: to manipulator frame
    Eigen::Affine3d mount_to_corrected_goal = camera_to_mount.inverse() * corrected_goal_rot;
    auto ikgen = robot_model_->search_all_ik_solutions(mount_to_corrected_goal, curr_joint_state_->position, sbpl::angles::to_radians(1.0));
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

        curr_joint_state_ = msg; // const everywhere else
    }
}

void ViservoControlExecutor::ar_markers_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    ROS_DEBUG("Received an AR marker message at %s", boost::posix_time::to_simple_string(msg->header.stamp.toBoost()).c_str());
    // scan the message to filter out ones that do not contain the markers we are tracking
    bool found = false;
    for (const auto& marker : msg->markers) {
        if (marker.id == attached_marker_.marker_id) {
            found = true;
            break;
        }
    }

    if (found) {
        last_ar_markers_msg_ = msg;
    }
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
    if (!curr_joint_state_) {
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
    std::vector<double> joint_vector = curr_joint_state_->position;

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
            msg_utils::download_param(ph_, "minimum_joint_velocities_deg", minimum_joint_velocities_dps) &&
            msg_utils::download_param(ph_, "kp", KP_) &&
            msg_utils::download_param(ph_, "ki", KI_) &&
            msg_utils::download_param(ph_, "kd", KD_)
    ; // LO'D!

    success &= deadband_joint_velocities_dps.size() == 7 && minimum_joint_velocities_dps.size() == 7;
    if (!success) {
        return false;
    }

    // convert to radians
    deadband_joint_velocities_rps_ = msg_utils::to_radians(deadband_joint_velocities_dps);
    minimum_joint_velocities_rps_ = msg_utils::to_radians(minimum_joint_velocities_dps);

    goal_rot_tolerance_ = sbpl::angles::to_radians(goal_rot_tolerance_);

    attached_marker_.link_to_marker = Eigen::Affine3d(
            Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
            Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

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
                sbpl::angles::ShortestAngleDistWithLimits(to_angle, from_angle, min_angle, max_angle) <
                sbpl::angles::to_radians(angle_threshold_degs);

        if (!safe_joint_command) {
            ROS_DEBUG("Next joint state too far away from current joint state (%0.3f degs - %0.3f degs > %0.3f degs)",
                sbpl::angles::to_degrees(to_angle),
                sbpl::angles::to_degrees(from_angle),
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
    traj_cmd.points[0].positions = curr_joint_state_->position;
    traj_cmd.points[0].velocities = std::vector<double>(robot_model_->joint_names().size(), 0.0);

    joint_command_pub_.publish(traj_cmd);
}

void ViservoControlExecutor::update_histogram()
{
    // keep track of which joints have moved in the wrong direction
    for (std::size_t i = 0; i < last_curr_.size(); ++i) {
        if (signf(curr_joint_state_->position[i] - last_curr_[i], sbpl::angles::to_radians(1.0)) !=
            signf(last_diff_[i], sbpl::angles::to_radians(1.0)))
        {
            ++misbehaved_joints_histogram_[i];
        }
    }
    ROS_INFO("Misbehaved joint histogram: %s", to_string(misbehaved_joints_histogram_).c_str());
}

KDL::FrameVel ViservoControlExecutor::compute_ee_velocity(const sensor_msgs::JointState& joint_state)
{
    KDL::JntArray q(kdl_chain_.getNrOfJoints());
    KDL::JntArray qdot(kdl_chain_.getNrOfJoints());
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        q(i, 0) = joint_state.position[i];
        qdot(i, 0) = joint_state.velocity[i];
    }

    KDL::JntArrayVel velocities(q, qdot);

    KDL::FrameVel fv_frame;
    fv_solver_->JntToCart(velocities, fv_frame);
    return fv_frame;
}

void ViservoControlExecutor::publish_triad_marker(
    const std::string& ns,
    const Eigen::Affine3d& transform,
    const std::string& frame)
{
    geometry_msgs::Vector3 scale;
    scale.x = 0.10;
    scale.y = 0.01;
    scale.z = 0.01;
    visualization_msgs::MarkerArray triad_marker = msg_utils::create_triad_marker_arr(scale);

    for (auto& marker : triad_marker.markers) {
        marker.header.frame_id = frame;
        marker.ns = ns;

        Eigen::Affine3d marker_transform;
        tf::poseMsgToEigen(marker.pose, marker_transform);
        marker_transform = transform * marker_transform;

        tf::poseEigenToMsg(marker_transform, marker.pose);
    }

    corrected_wrist_goal_pub_.publish(triad_marker);
}

bool ViservoControlExecutor::lookup_transform(
    const std::string& from,
    const std::string& to,
    const ros::Time& time,
    Eigen::Affine3d& out)
{
    tf::StampedTransform transform;
    try {
        listener_.lookupTransform(from, to, time, transform);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Unable to lookup transform from %s to %s (%s)", to.c_str(), from.c_str(), ex.what());
        return false;
    }

    const double tx = transform.getOrigin().x();
    const double ty = transform.getOrigin().y();
    const double tz = transform.getOrigin().z();

    double qw = transform.getRotation().w();
    double qx = transform.getRotation().x();
    double qy = transform.getRotation().y();
    double qz = transform.getRotation().z();

    out = Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);
    return true;
}

bool ViservoControlExecutor::choose_best_ik_solution(
    const Eigen::Affine3d& ee_transform,
    const std::vector<double>& from,
    std::vector<double>& to,
    std::string& why)
{
    double ik_search_res = sbpl::angles::to_radians(1.0);
    hdt::IKSolutionGenerator ikgen = robot_model_->search_all_ik_solutions(ee_transform, from, ik_search_res);
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
        why = "Failed to compute IK solution";
        return false;
    }

    ROS_INFO("Picking best IK solution out of %d", num_solutions_found);

    bool found_ik = false;
    double best_dist = -1.0;
    std::vector<double> chosen_solution;
    for (const std::vector<double>& sol : ik_solutions) {
        if (safe_joint_delta(from, sol)) {
            if (!found_ik) {
                found_ik = true;
                best_dist = hdt::ComputeJointStateL2NormSqrd(from, sol);
                chosen_solution = sol;
            }
            else {
                double dist = hdt::ComputeJointStateL2NormSqrd(from, sol);
                if (dist < best_dist) {
                    best_dist = dist;
                    chosen_solution = sol;
                }
            }
        }
    }

    if (!found_ik) {
        why = "No safe IK solution";
        return false;
    }

    to = std::move(chosen_solution);
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viservo_control_executor");
    return ViservoControlExecutor().run();
}
