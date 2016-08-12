#include <chrono>
#include <RADMessages.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sbpl_geometry_utils/utils.h>
#include <sensor_msgs/JointState.h>
#include <rcta/ControllerDiagnosticStatus.h>
#include "manipulator_interface_sim_ros.h"

namespace hdt
{

ManipulatorInterfaceSimROS::ManipulatorInterfaceSimROS() :
    ManipulatorInterfaceROS(),
    init_error_(ManipulatorError::NO_ERROR()),
    nh_(),
    ph_("~"),
    follow_joint_traj_feedback_pub_(),
    joint_states_pub_(),
    diagnostic_status_pub_(),
    joint_traj_sub_(),
    true_joint_positions_(joint_names().size(), 0.0),
    noisy_joint_positions_(joint_names().size(), 0.0),
    joint_position_command_(true_joint_positions_),
    joint_velocities_(joint_names().size(), 0.0),
    manip_params_(),
    rd_(),
    rng_(rd_()),
    as_(),
    action_server_name_("teleport_hdt_command")
{
    int i = 0;
    for (const std::string& joint_name : joint_names()) {
        joint_indices_[joint_name] = i++;
    }
}

ManipulatorInterfaceSimROS::~ManipulatorInterfaceSimROS()
{
}

ManipulatorInterfaceROS::RunResult ManipulatorInterfaceSimROS::run()
{
    if (!init()) {
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(50.0);
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    start = std::chrono::high_resolution_clock::now();
    end = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        end = std::chrono::high_resolution_clock::now();

        // move towards set point
        double dt = std::chrono::duration<double>(end - start).count();

        // create joint velocities to move towards the given command based off of the last noisy joint state that was published

        // TODO: this multiplier affects at what angle distance away from the set point we will command the arm to
        // drive at maximum speed (on a per-angle basis). To automatically come up with this parameter, calculate this
        // multiplier based on the joint accelerations such that we drive at maximum speed until we must command
        // something slower to be able to reach the set point with zero velocity.
        const double speed_multiplier = 5.0;
        joint_velocities_ = anglediff(joint_position_command_, noisy_joint_positions_, min_limits(), max_limits());
        joint_velocities_ = mul(speed_multiplier, joint_velocities_);
        clampall(joint_velocities_, min_velocity_limits(), max_velocity_limits());

        true_joint_positions_ = sum(true_joint_positions_, mul(dt, joint_velocities_));
        noisy_joint_positions_ = sum(noisy_joint_positions_, mul(dt, joint_velocities_));

        ROS_DEBUG("True Joint Positions: %s", to_string(true_joint_positions_).c_str());
        ROS_DEBUG("Noisy Joint Positions: %s", to_string(noisy_joint_positions_).c_str());

        // add noise

        std::vector<std::normal_distribution<>> dists;
        const double variance_deg = 0.05;
        for (const double jpos : true_joint_positions_) {
            dists.push_back(std::normal_distribution<>(jpos, sbpl::utils::ToRadians(variance_deg)));
        }

        std::vector<double> perturbations(true_joint_positions_.size());
        for (size_t i = 0; i < perturbations.size(); ++i) {
            perturbations[i] = dists[i](rng_);
        }

        noisy_joint_positions_ = sum(true_joint_positions_, perturbations);

        ////////////////////////////////////////////////////////////////////////
        // Publish hdt/ControllerDiagnosticStatus to "hdt_diagnostics"
        ////////////////////////////////////////////////////////////////////////

        static int seqno = 0;
        rcta::ControllerDiagnosticStatus status;
        status.header.stamp = now;
        status.header.seq = seqno++;
        status.header.frame_id = "";
        status.reset_occurred = false;
        status.is_controllable = true;
        status.have_supply = true;
        status.have_comms = true;
        status.update_frequency = 50.0; // TODO: grab this from loop rate
        status.joint_status.clear();
        status.joint_status.resize(joint_names().size());

        std::vector<RAD::SWState> sw_states(joint_names().size(), RAD::SW_NOS);
        std::vector<RAD::BITStatus> bit_stati(joint_names().size(), RAD::BIT_NO_ERROR);
        std::vector<float> temperatures(joint_names().size(), 30.0);
        std::vector<bool> temperature_stati(joint_names().size(), false);
        std::vector<bool> motor_volt_stati(joint_names().size(), false);
        std::vector<bool> motor_current_stati(joint_names().size(), false);
        std::vector<bool> comms_fault_stati(joint_names().size(), false);
        std::vector<bool> misc_fault_stati(joint_names().size(), false);

        for (size_t i = 0; i < joint_names().size(); ++i) {
            status.joint_status[i].temperature          = temperatures[i];
            status.joint_status[i].temperature_status   = temperature_stati[i];
            status.joint_status[i].voltage_status       = motor_volt_stati[i];
            status.joint_status[i].current_status       = motor_current_stati[i];
            status.joint_status[i].comms_status         = comms_fault_stati[i];
            status.joint_status[i].fault_status         = misc_fault_stati[i];
        }

        diagnostic_status_pub_.publish(status);

        ////////////////////////////////////////////////////////////////////////
        // Publish sensor_msgs/JointState to "joint_states"
        ////////////////////////////////////////////////////////////////////////

        static int js_seqno = 0;
        sensor_msgs::JointState curr_joint_state;
        curr_joint_state.header.seq = js_seqno++;
        curr_joint_state.header.stamp = now;
        curr_joint_state.header.frame_id = "";
        curr_joint_state.name = joint_names();
        curr_joint_state.position.resize(joint_names().size());
        curr_joint_state.velocity.resize(joint_names().size());
        curr_joint_state.effort.resize(joint_names().size());
        for (size_t i = 0; i < joint_names().size(); ++i) {
            curr_joint_state.position[i] = noisy_joint_positions_[i];
            curr_joint_state.velocity[i] = joint_velocities_[i];
            curr_joint_state.effort[i] = 0.0;
        }

        joint_states_pub_.publish(curr_joint_state);

        ////////////////////////////////////////////////////////////////////////
        // Publish control_msgs/FollowJointTrajectoryFeedback to "feedback_states"
        ////////////////////////////////////////////////////////////////////////

        control_msgs::FollowJointTrajectoryFeedback control_state;
        control_state.header.stamp = now;
        control_state.joint_names = joint_names();
        control_state.actual.positions = noisy_joint_positions_;
        follow_joint_traj_feedback_pub_.publish(control_state);

        ros::spinOnce();
        start = std::chrono::high_resolution_clock::now();
        loop_rate.sleep();
    }

    return SUCCESS;
}

bool ManipulatorInterfaceSimROS::init()
{
    std::string manip_config_fname;
    if (!ph_.getParam("hdt_manipulator_config", manip_config_fname)) {
        ROS_ERROR("Failed to retrieve '~hdt_manipulator_config'");
        return false;
    }

    if (!read_manip_params(manip_config_fname, manip_params_)) {
        ROS_ERROR("Failed to parse '%s' for manipulator config", manip_config_fname.c_str());
        return false;
    }

    follow_joint_traj_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states_raw", 1);
    diagnostic_status_pub_ = nh_.advertise<rcta::ControllerDiagnosticStatus>("hdt_diagnostics", 1);
    joint_traj_sub_ = nh_.subscribe("command", 1, &ManipulatorInterfaceSimROS::joint_trajectory_callback, this);

    as_.reset(new TeleportHDTCommandActionServer(action_server_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate action server '%s'", action_server_name_.c_str());
        return false;
    }

    auto goal_cb = boost::bind(&ManipulatorInterfaceSimROS::goal_callback, this);
    auto preempt_cb = boost::bind(&ManipulatorInterfaceSimROS::preempt_callback, this);
    as_->registerGoalCallback(goal_cb);
    as_->registerPreemptCallback(preempt_cb);

    ROS_INFO("Starting action server '%s", action_server_name_.c_str());
    as_->start();
    ROS_INFO("Started action server");

    return true;
}

void ManipulatorInterfaceSimROS::joint_trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if (!check_joints(msg->joint_names)) {
        ROS_WARN("Received joint trajectory for invalid set of joints");
        return;
    }

    if (!msg->points.empty()) {
        if (msg->points.size() > 1) {
            ROS_WARN("Ignoring all points from Joint Trajectory except the final point");
        }

        const trajectory_msgs::JointTrajectoryPoint& final_point = msg->points.back();
        if (final_point.positions.size() != joint_names().size()) {
            ROS_ERROR("Insufficient joint position information");
            return;
        }

        std::vector<double> target;
        target.reserve(joint_names().size());

        for (const std::string& joint_name : joint_names()) {
            int index = find_joint_index(joint_name, *msg);
            if (index < 0) {
                ROS_ERROR("Failed to find position information for joint %s", joint_name.c_str());
                return;
            }
            else {
                target.push_back(final_point.positions[index]);
            }
        }

        joint_position_command_ = std::move(target);
    }
}

double ManipulatorInterfaceSimROS::get_max_velocity(const std::string& joint_name) const
{
    int index = get_joint_index(joint_name);
    if (index < 0) {
        return -1.0;
    }
    else {
        if (manip_params_.Joint(index).Name() != joint_name) {
            return -1.0;
        }
        else {
            return manip_params_.Joint(index).MaxVelocity();
        }
    }
}

int ManipulatorInterfaceSimROS::get_joint_index(const std::string& joint_name) const
{
    auto it = joint_indices_.find(joint_name);
    if (it == joint_indices_.end()) {
        return -1;
    }
    else {
        return it->second;
    }
}

std::vector<double> ManipulatorInterfaceSimROS::neg(const std::vector<double>& values) const
{
    std::vector<double> res(values.size());
    for (size_t i = 0; i < res.size(); ++i) {
        res[i] = -values[i];
    }
    return res;
}

std::vector<double> ManipulatorInterfaceSimROS::mul(double scalar, const std::vector<double>& values) const
{
    std::vector<double> res(values.size());
    for (size_t i = 0; i < res.size(); ++i) {
        res[i] = scalar * values[i];
    }
    return res;
}

std::vector<double> ManipulatorInterfaceSimROS::sum(const std::vector<double>& u, const std::vector<double>& v) const
{
    if (u.size() != v.size()) {
        ROS_WARN("Attempt to add vectors of different sizes");
        return { };
    }

    std::vector<double> res(u.size());
    for (size_t i = 0; i < res.size(); ++i) {
        res[i] = u[i] + v[i];
    }
    return res;
}

std::vector<double> ManipulatorInterfaceSimROS::anglediff(
    const std::vector<double>& u,
    const std::vector<double>& v,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits) const
{
    if (u.size() != v.size()) {
        ROS_WARN("Attempt to anglediff vectors of different sizes");
        return { };
    }

    std::vector<double> res(u.size());
    for (size_t i = 0; i < res.size(); ++i) {
        double clamp_u = u[i];
        double clamp_v = v[i];
        clamp(clamp_u, min_limits[i], max_limits[i]);
        clamp(clamp_v, min_limits[i], max_limits[i]);
        res[i] = sbpl::utils::ShortestAngleDiffWithLimits(clamp_u, clamp_v, min_limits[i], max_limits[i]);
    }

    return res;
}

const std::vector<double>& ManipulatorInterfaceSimROS::min_limits() const
{
    static std::vector<double> limits;
    if (limits.size() != manip_params_.NumJoints()) {
        limits.clear();
        limits.reserve(manip_params_.NumJoints());
        for (size_t i = 0; i < manip_params_.NumJoints(); ++i) {
            limits.push_back(manip_params_.Joint(i).MinPosition());
        }
    }
    return limits;
}

const std::vector<double>& ManipulatorInterfaceSimROS::max_limits() const
{
    static std::vector<double> limits;
    if (limits.size() != manip_params_.NumJoints()) {
        limits.clear();
        limits.reserve(manip_params_.NumJoints());
        for (size_t i = 0; i < manip_params_.NumJoints(); ++i) {
            limits.push_back(manip_params_.Joint(i).MaxPosition());
        }
    }
    return limits;
}

const std::vector<double>& ManipulatorInterfaceSimROS::min_velocity_limits() const
{
    static std::vector<double> limits;
    if (limits.size() != manip_params_.NumJoints()) {
        limits.clear();
        limits.reserve(manip_params_.NumJoints());
        for (size_t i = 0; i < manip_params_.NumJoints(); ++i) {
            limits.push_back(-manip_params_.Joint(i).MaxVelocity());
        }
    }
    return limits;
}

const std::vector<double>& ManipulatorInterfaceSimROS::max_velocity_limits() const
{
    static std::vector<double> limits;
    if (limits.size() != manip_params_.NumJoints()) {
        limits.clear();
        limits.reserve(manip_params_.NumJoints());
        for (size_t i = 0; i < manip_params_.NumJoints(); ++i) {
            limits.push_back(manip_params_.Joint(i).MaxVelocity());
        }
    }
    return limits;
}

void ManipulatorInterfaceSimROS::goal_callback()
{
    auto current_goal = as_->acceptNewGoal();
    
    if (current_goal->joint_state.position.size() != 7) {
        as_->setAborted();
        return;
    }

    true_joint_positions_ = current_goal->joint_state.position;
    noisy_joint_positions_ = current_goal->joint_state.position;
    joint_velocities_ = std::vector<double>(7, 0.0);
    joint_position_command_ = current_goal->joint_state.position;
    as_->setSucceeded();
}

void ManipulatorInterfaceSimROS::preempt_callback()
{
}

} // namespace hdt
