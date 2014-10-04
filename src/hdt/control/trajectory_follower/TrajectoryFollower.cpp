#include "TrajectoryFollower.h"

#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/stringifier/stringifier.h>
#include <sbpl_geometry_utils/utils.h>

TrajectoryFollower::TrajectoryFollower() :
    nh_(),
    ph_("~"),
    as_(),
    action_server_name_("arm_controller/joint_trajectory_action"),
    joint_traj_pub_()
{

}

TrajectoryFollower::~TrajectoryFollower()
{
    as_->shutdown();
}

TrajectoryFollower::MainResult TrajectoryFollower::run()
{
    if (!initialize()) {
        ROS_ERROR("Failed to initialize");
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });

        ros::spinOnce();

        if (!as_->isActive()) {
            continue;
        }

        // delay execution 1s from receiving the goal
        ros::Time now = ros::Time::now();
        if (now < execution_start_time_) {
            continue;
        }

        // work on current goal
        ros::Time time_along_path = ros::Time(0) + (now - execution_start_time_);

        // finish
        if (time_along_path > ros::Time(0) + current_goal_.trajectory.points.back().time_from_start) {
            as_->setSucceeded();
            continue;
        }

        trajectory_msgs::JointTrajectoryPoint traj_pt;
        if (!get_point_at_time(current_goal_, time_along_path, traj_pt)) {
            ROS_WARN("Failed to get point along trajectory at time %0.6f", time_along_path.toSec());
            continue;
        }

        ROS_INFO("Publishing intermediate point");
        ROS_INFO("  Positions: %s", to_string(traj_pt.positions).c_str());
        ROS_INFO("  Velocities: %s", to_string(traj_pt.velocities).c_str());
        ROS_INFO("  Accelerations: %s", to_string(traj_pt.accelerations).c_str());
        ROS_INFO("  Time From Start: %0.6f", traj_pt.time_from_start.toSec());

        trajectory_msgs::JointTrajectory joint_traj;
        joint_traj.header.seq = 0;
        joint_traj.header.stamp = now;
        joint_traj.header.frame_id = "";
        joint_traj.joint_names = robot_model_->joint_names();
        joint_traj.points.push_back(traj_pt);
        joint_traj_pub_.publish(joint_traj);
    }

    return SUCCESS;
}

bool TrajectoryFollower::initialize()
{
    // create robot models
    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from the URDF");
        return false;
    }

    // todo: read control parameters, if any

    // fire up the action server
    as_.reset(new FollowJointTrajectoryActionServer(action_server_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Follower Joint Trajectory Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&TrajectoryFollower::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&TrajectoryFollower::preempt_callback, this));

    as_->start();

    // fire up publishers
    joint_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 5);

    return true;
}

void TrajectoryFollower::goal_callback()
{
    auto goal = as_->acceptNewGoal();

    ROS_INFO("Accepted new goal");
    ROS_INFO("  trajectory:");
    ROS_INFO("    header:");
    ROS_INFO("      seq: %u", goal->trajectory.header.seq);
    ROS_INFO("      stamp: %s", boost::posix_time::to_simple_string(goal->trajectory.header.stamp.toBoost()).c_str());
    ROS_INFO("      frame_id: %s", goal->trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %s", to_string(goal->trajectory.joint_names).c_str());
    ROS_INFO("    points: [%zd points]", goal->trajectory.points.size());
    ROS_INFO("  path_tolerance");
    for (const auto& tolerance : goal->path_tolerance) {
        ROS_INFO("    name: %s, position: %0.6f, velocity: %0.6f, acceleration: %0.6f", tolerance.name.c_str(), tolerance.position, tolerance.velocity, tolerance.acceleration);
    }
    ROS_INFO("  goal_tolerance");
    for (const auto& tolerance : goal->goal_tolerance) {
        ROS_INFO("    name: %s, position: %0.6f, velocity: %0.6f, acceleration: %0.6f", tolerance.name.c_str(), tolerance.position, tolerance.velocity, tolerance.acceleration);
    }
    ROS_INFO("  goal_time_tolerance: %0.3f", goal->goal_time_tolerance.toSec());

    if (!check_goal(*goal)) {
        as_->setAborted();
    }

    // retime trajectory
    current_goal_ = retime_trajectory(*goal);

    execution_start_time_ = ros::Time::now() + ros::Duration(1.0);
}

void TrajectoryFollower::preempt_callback()
{

}

bool TrajectoryFollower::check_goal(const control_msgs::FollowJointTrajectoryGoal& goal) const
{
    // todo: check for correct joint names and joint position limits
    return goal.trajectory.joint_names.size() == robot_model_->joint_names().size();
}

control_msgs::FollowJointTrajectoryGoal
TrajectoryFollower::retime_trajectory(const control_msgs::FollowJointTrajectoryGoal& goal)
{
    ROS_INFO("Retiming trajectory");
    control_msgs::FollowJointTrajectoryGoal retime_goal;

    retime_goal.trajectory.header = goal.trajectory.header;
    retime_goal.trajectory.joint_names = goal.trajectory.joint_names;
    retime_goal.trajectory.points.reserve(goal.trajectory.points.size());
    ROS_INFO("Joint Trajectory:");
    for (size_t i = 0; i < goal.trajectory.points.size(); ++i)
    {
        const auto& point = goal.trajectory.points[i];

        if (i == 0) {
            trajectory_msgs::JointTrajectoryPoint first_point;
            first_point.positions = point.positions;
            first_point.velocities = std::vector<double>(robot_model_->joint_names().size(), 0.0);
            first_point.accelerations; // todo: probably zero, currently ignored by controller anyway
            first_point.time_from_start = ros::Duration(0.0);
            retime_goal.trajectory.points.push_back(first_point);
        }
        else {
            const auto& prev_point = goal.trajectory.points[i - 1];
            const auto& prev_retime_point = retime_goal.trajectory.points.back();

            trajectory_msgs::JointTrajectoryPoint new_point;
            new_point.positions = point.positions; // preserve joint positions

            // todo:
            // 1. calculate the amount of time for each joint to reach this position from the previous position
            // 2. take the maximum amount of time required and use it to modulate the velocity at each joint
            // 3. add that maximum amount of time to the previous timestamp to get the time from the start

            double required_wp_time_s = 0.0;
            for (size_t jidx = 0; jidx < robot_model_->num_joints(); ++jidx) {
                double min_limit = robot_model_->min_limits()[jidx];
                double max_limit = robot_model_->max_limits()[jidx];
                double angle_dist = sbpl::utils::ShortestAngleDistWithLimits(point.positions[jidx], prev_point.positions[jidx], min_limit, max_limit);
                double max_velocity_rps = robot_model_->max_velocity_limits()[jidx];
                double required_joint_time_s = fabs(angle_dist / max_velocity_rps);
                required_wp_time_s = std::max(required_wp_time_s, required_joint_time_s);
            }

            ROS_INFO("    required time between waypoints: %0.6f", required_wp_time_s);

            new_point.velocities.resize(robot_model_->num_joints());
            for (size_t jidx = 0; jidx < robot_model_->num_joints(); ++jidx) {
                double min_limit = robot_model_->min_limits()[jidx];
                double max_limit = robot_model_->max_limits()[jidx];
                double angle_dist_rad = sbpl::utils::ShortestAngleDistWithLimits(point.positions[jidx], prev_point.positions[jidx], min_limit, max_limit);
                double max_velocity_rps = robot_model_->max_velocity_limits()[jidx];
                new_point.velocities[jidx] = (required_wp_time_s == 0.0) ? (0.0) : (std::min(angle_dist_rad / required_wp_time_s, max_velocity_rps));
            }

            new_point.accelerations; // todo: maybe translate this to the amount of torque to apply?

            new_point.time_from_start = prev_retime_point.time_from_start + ros::Duration(required_wp_time_s); //

            retime_goal.trajectory.points.push_back(new_point);
        }

        ROS_INFO("  point %3zd", i);
        ROS_INFO("    positions: %s", to_string(retime_goal.trajectory.points.back().positions).c_str());
        ROS_INFO("    velocities: %s", to_string(retime_goal.trajectory.points.back().velocities).c_str());
        ROS_INFO("    accelerations: %s", to_string(retime_goal.trajectory.points.back().accelerations).c_str());
        ROS_INFO("    time from start: %0.6f", retime_goal.trajectory.points.back().time_from_start.toSec());
    }

    retime_goal.path_tolerance; // todo: overriden from config
    retime_goal.goal_tolerance; // todo: overriden from config
    retime_goal.goal_time_tolerance = goal.goal_time_tolerance;

    ROS_INFO("Finished retiming trajectory with %zd points", retime_goal.trajectory.points.size());
    return retime_goal;
}

bool TrajectoryFollower::get_point_at_time(
    const control_msgs::FollowJointTrajectoryGoal& joint_traj,
    const ros::Time& time_from_start,
    trajectory_msgs::JointTrajectoryPoint& traj_pt) const
{
    if (joint_traj.trajectory.points.empty()) {
        ROS_WARN("Cannot return intermediate trajectory point for empty trajectories");
        return false;
    }

    for (size_t pidx = 0; pidx < joint_traj.trajectory.points.size(); ++pidx) {
        const auto& point = joint_traj.trajectory.points[pidx];
        if (ros::Time(0) + point.time_from_start >= time_from_start) {
            // interpolate between this point and the next point
            if (pidx == joint_traj.trajectory.points.size() - 1) {
                 traj_pt = joint_traj.trajectory.points.back();
                 return true;
            }

            const auto& next_point = joint_traj.trajectory.points[pidx + 1];
            double time_between_points_s = (next_point.time_from_start - point.time_from_start).toSec();
            double time_along_waypoint_s = (point.time_from_start - point.time_from_start).toSec();
            double alpha = (time_between_points_s == 0.0) ? (0.0) : ( time_along_waypoint_s / time_between_points_s);

            // interpolate between joint positions and velocities
            auto interp = [](const std::vector<double>& from, const std::vector<double>& to, double alpha)
            {
                if (from.size() != to.size()) {
                    return std::vector<double>();
                }

                std::vector<double> interm((size_t)from.size());
                for (size_t i = 0; i < from.size(); ++i) {
                    interm[i] = (1.0 - alpha) * from[i] + alpha * to[i];
                }

                return interm;
            };

            std::vector<double> interm_position = interp(point.positions, next_point.positions, alpha);
            std::vector<double> interm_velocities = interp(point.velocities, next_point.velocities, alpha);
            if (interm_position.empty() || interm_velocities.empty()) {
                ROS_WARN("Failed to interpolate joint positions or velocities");
                return false;
            }

            trajectory_msgs::JointTrajectoryPoint interm_point;
            interm_point.positions = std::move(interm_position);
            interm_point.velocities = std::move(interm_velocities);
            interm_point.time_from_start = time_from_start - ros::Time(0);
            traj_pt = interm_point;
            return true;
        }
    }

    traj_pt = joint_traj.trajectory.points.back();
    return true;
}
