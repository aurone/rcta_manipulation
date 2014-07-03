#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <RADMessages.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <hdt/DiagnosticStatus.h>
#include "manipulator_interface_live_ros.h"

template <typename T, typename S>
std::vector<S> coerce(const std::vector<T>& v)
{
    std::vector<S> vec(v.size());
    for (const T& e : v) {
        vec.emplace_back(S(e));
    }
    return vec;
}

template <typename T, typename S>
void coerce(const std::vector<T>& u, std::vector<S>& v)
{
    v.clear();
    v.reserve(u.size());
    for (const T& e : u) {
        v.emplace_back(S(e));
    }
}

namespace hdt
{

ManipulatorInterfaceLiveROS::ManipulatorInterfaceLiveROS() :
    ManipulatorInterfaceROS(),
    manip_(),
    init_error_(ManipulatorError::NO_ERROR()),
    nh_(),
    ph_("~"),
    follow_joint_traj_feedback_pub_(),
    joint_states_pub_(),
    joint_traj_sub_(),
    initialized_(false),
    estopped_(false),
    robot_model_()
{
}

ManipulatorInterfaceLiveROS::~ManipulatorInterfaceLiveROS()
{
    if (initialized_) {
        manip_.shutdown();
    }
}

ManipulatorInterfaceROS::RunResult ManipulatorInterfaceLiveROS::run()
{
    if (!init()) {
        return FAILED_TO_INITIALIZE;
    }

    {
        trajectory_msgs::JointTrajectory* fake_msg_ptr = new trajectory_msgs::JointTrajectory;
        fake_msg_ptr->joint_names = joint_names();
        fake_msg_ptr->points.resize(1);
        fake_msg_ptr->points.front().positions = std::vector<double>(joint_names().size(), 0.0);

        std::vector<float> raw_joint_positions(manip_.getNumJoints());
        ManipulatorError error(ManipulatorError::NO_ERROR());
        error = manip_.getPosition(raw_joint_positions);
        if (error != ManipulatorError::NO_ERROR()) {
            ROS_ERROR("Failed to send initial joint command");
            return FAILED_TO_INITIALIZE;
        }
        coerce(raw_joint_positions, fake_msg_ptr->points.front().positions);

        last_command_.reset(fake_msg_ptr);
        joint_trajectory_callback(last_command_);
    }

    ros::Rate loop_rate(50.0);
    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        ////////////////////////////////////////////////////////////////////////
        // Publish hdt/DiagnosticStatus to "hdt_diagnostics"
        ////////////////////////////////////////////////////////////////////////

        static int seqno = 0;
        hdt::DiagnosticStatus status;
        status.header.stamp = now;
        status.header.seq = seqno++;
        status.header.frame_id = "";
        status.reset_occurred = manip_.resetOccurred();
        status.is_controllable = manip_.isControllable();
        status.have_supply = manip_.haveSupply();
        status.have_comms = manip_.haveCommunication();
        status.update_frequency = manip_.getLastUpdateFrequency();
        status.joint_status.clear();
        status.joint_status.resize(joint_names().size());

        // NOTE: these vectors must be preallocated before being sent to the associated HDTManipulatorInterface::get* commands
        std::vector<RAD::SWState> sw_states(joint_names().size());
        std::vector<RAD::BITStatus> bit_stati(joint_names().size(), RAD::BIT_CRITICAL_ERROR);
        std::vector<float> temperatures(joint_names().size(), -1.0);
        std::vector<bool> temperature_stati(joint_names().size(), true);
        std::vector<bool> motor_volt_stati(joint_names().size(), true);
        std::vector<bool> motor_current_stati(joint_names().size(), true);
        std::vector<bool> comms_fault_stati(joint_names().size(), true);
        std::vector<bool> misc_fault_stati(joint_names().size(), true);

        manip_.getSoftwareState(sw_states);
        manip_.getBITStatus(bit_stati);
        manip_.getMotorTemperature(temperatures);
        manip_.getMotorTempFaultStatus(temperature_stati);
        manip_.getMotorVoltFaultStatus(motor_volt_stati);
        manip_.getMotorCurrentFaultStatus(motor_current_stati);
        manip_.getCommsFaultStatus(comms_fault_stati);
        manip_.getMiscFaultStatus(misc_fault_stati);

        if (temperatures.size() != joint_names().size() ||
            sw_states.size() != joint_names().size() ||
            bit_stati.size() != joint_names().size() ||
            temperature_stati.size() != joint_names().size() ||
            motor_volt_stati.size() != joint_names().size() ||
            motor_current_stati.size() != joint_names().size() ||
            comms_fault_stati.size() != joint_names().size() ||
            misc_fault_stati.size() != joint_names().size())
        {
            ROS_ERROR("Status vector does not contain information for %zd joints", joint_names().size());
            ROS_ERROR("    len(Temperature): %zd",          temperatures.size());
            ROS_ERROR("    len(Software State): %zd",       sw_states.size());
            ROS_ERROR("    len(Bit State): %zd",            bit_stati.size());
            ROS_ERROR("    len(Temperature Status): %zd",   temperature_stati.size());
            ROS_ERROR("    len(Motor Volt Status): %zd",    motor_volt_stati.size());
            ROS_ERROR("    len(Motor Current Status): %zd", motor_current_stati.size());
            ROS_ERROR("    len(Comms Fault Status): %zd",   comms_fault_stati.size());
            ROS_ERROR("    len(Misc Fault Status): %zd",    misc_fault_stati.size());
        }

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

        std::vector<float> raw_joint_positions(manip_.getNumJoints());
        std::vector<float> raw_joint_velocities(manip_.getNumJoints());
        std::vector<float> raw_joint_torques(manip_.getNumJoints());
        std::vector<float> raw_joint_currents(manip_.getNumJoints());

        ManipulatorError error(ManipulatorError::NO_ERROR());
        error = manip_.getPosition(raw_joint_positions);

        if (manip_.getPosition(raw_joint_positions) != ManipulatorError::NO_ERROR() ||
            manip_.getVelocity(raw_joint_velocities) != ManipulatorError::NO_ERROR() ||
            manip_.getTorque(raw_joint_torques) != ManipulatorError::NO_ERROR() ||
            manip_.getMotorCurrent(raw_joint_currents) != ManipulatorError::NO_ERROR())
        {
            ROS_WARN("Failed to get joint position information (%s)", to_string(error).c_str());
        }
        else {
            static int js_seqno = 0;
            sensor_msgs::JointState curr_joint_state;
            curr_joint_state.header.seq = js_seqno++;
            curr_joint_state.header.stamp = now;
            curr_joint_state.header.frame_id = "";
            curr_joint_state.name = joint_names();
            coerce(raw_joint_positions, curr_joint_state.position);
            coerce(raw_joint_velocities, curr_joint_state.velocity);
            coerce(raw_joint_torques, curr_joint_state.effort);

            joint_states_pub_.publish(curr_joint_state);
        }

        ////////////////////////////////////////////////////////////////////////
        // Publish control_msgs/FollowJointTrajectoryFeedback to "feedback_states"
        ////////////////////////////////////////////////////////////////////////

        control_msgs::FollowJointTrajectoryFeedback control_state;
        control_state.header.stamp = now;
        control_state.joint_names = joint_names();
        coerce(raw_joint_positions, control_state.actual.positions);
        follow_joint_traj_feedback_pub_.publish(control_state);

        joint_trajectory_callback(last_command_);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return SUCCESS;
}

bool ManipulatorInterfaceLiveROS::init()
{
    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    if (!robot_model_.load(urdf_string)) {
        ROS_ERROR("Failed to load Robot Model from URDF");
        return false;
    }

    ROS_INFO("Removing esd_usb2 kernel module");
    int ret = system("modprobe -r esd_usb2");
    if (ret) {
        ROS_ERROR("'modprobe -r esd_usb2' exited with return code %d", ret);
        return false;
    }
    ros::Duration(1.0).sleep();
    ROS_INFO("Reinserting esd_usb2 kernel module");
    ret = system("modprobe -i esd_usb2");
    if (ret) {
        ROS_ERROR("'modprobe -i esd_usb2' exited with return code %d", ret);
        return false;
    }

    std::string params_fname;
    if (!ph_.getParam("hdt_manipulator_params", params_fname)) {
        ROS_ERROR("Failed to retrieve '~hdt_manipulator_params");
        return false;
    }

    ManipulatorParameters manip_params;
    ManipulatorError error = manip_params.initialize(params_fname);
    if (error != ManipulatorError::NO_ERROR() || !manip_params.isInitialized()) {
        ROS_ERROR("Failed to parse '%s' for manipulator config (%s)", params_fname.c_str(), to_string(error).c_str());
        return false;
    }

    // cross-check the joint limits from file with the joint limits from the urdf
    if (!assert_joint_limits(manip_params)) {
        ROS_ERROR("Joint limits disagreement between URDF and HDT Parameters file");
        return false;
    }

    init_error_ = manip_.initialize(manip_params);
    if (!manip_.isInitialized()) {
        ROS_ERROR("Failed to initialize HDT Manipulator Interface");
        return false;
    }

    follow_joint_traj_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states_raw", 1);
    diagnostic_status_pub_ = nh_.advertise<hdt::DiagnosticStatus>("hdt_diagnostics", 1);
    joint_traj_sub_ = nh_.subscribe("command", 5, &ManipulatorInterfaceLiveROS::joint_trajectory_callback, this);
    estop_sub_ = nh_.subscribe("hdt_estop", 1, &ManipulatorInterfaceLiveROS::emergency_stop_callback, this);
    clear_estop_sub_ = nh_.subscribe("clear_hdt_estop", 1, &ManipulatorInterfaceLiveROS::clear_emergency_stop_callback, this);
    ack_reset_sub_ = nh_.subscribe("acknowledge_hdt_reset", 1, &ManipulatorInterfaceLiveROS::acknowledge_reset_callback, this);

    ROS_INFO("Initialization successful");

    initialized_ = true;
    return initialized_; // easily detect if we forgot to set initialized_
}

void ManipulatorInterfaceLiveROS::joint_trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if (estopped_) {
        ROS_WARN("Rejecting command due to Emergency Stop");
        return;
    }

    last_command_ = msg;

    if (!check_joints(msg->joint_names)) {
        ROS_WARN("Received joint trajectory for invalid set of joints");
        return;
    }

    if (!msg->points.empty()) {
        std::vector<double> target = extract_target_command(*msg); 

        if (target.empty()) { // errors logged within extract_target_command
            return;
        }

        const double velocity_limit_dps = 20.0;
        double velocity_limit_rps = velocity_limit_dps * M_PI / 180.0;
        ROS_WARN_ONCE("Using hardcoded velocity limit of %0.3f deg/s (%0.3f rad/s)", velocity_limit_dps, velocity_limit_rps);
        std::vector<double> joint_velocity_limits(joint_names().size(), velocity_limit_rps);

        std::vector<float> float_positions, float_velocities;
        coerce(target, float_positions);
        coerce(joint_velocity_limits, float_velocities);
        target_position_and_velocity(float_positions, float_velocities);
    }
}

void ManipulatorInterfaceLiveROS::emergency_stop_callback(const hdt::EmergencyStop::ConstPtr& msg)
{
    estopped_ = true;
    halt();
}

void ManipulatorInterfaceLiveROS::clear_emergency_stop_callback(const hdt::ClearEmergencyStop::ConstPtr& msg)
{
    estopped_ = false;
}

void ManipulatorInterfaceLiveROS::target_position_and_velocity(
    const std::vector<float>& position,
    const std::vector<float>& velocity)
{
    ROS_DEBUG("Sending joint positions = %s, joint velocities = %s!", to_string(position).c_str(), to_string(velocity).c_str());
    manip_.setTargetPositionVelocity(position, velocity);
    manip_.setMotorCurrentLimit(std::vector<float>(joint_names().size(), 10.0)); // 10.0 taken from HDTManipulator.cpp
}

void ManipulatorInterfaceLiveROS::halt()
{
    std::vector<float> raw_joint_positions(manip_.getNumJoints());
    ManipulatorError error(ManipulatorError::NO_ERROR());
    error = manip_.getPosition(raw_joint_positions);
    if (error != ManipulatorError::NO_ERROR()) {
        ROS_ERROR("Failed to get current joint positions. Halting at last command");
        std::vector<double> last_target = extract_target_command(*last_command_);
        coerce(last_target, raw_joint_positions);
    }

    std::vector<float> zero_joint_velocity(manip_.getNumJoints(), 0.0f);
    target_position_and_velocity(raw_joint_positions, zero_joint_velocity);
}

std::vector<double>
ManipulatorInterfaceLiveROS::extract_target_command(const trajectory_msgs::JointTrajectory& command_msg) const
{
    if (command_msg.points.size() > 1) {
        ROS_WARN("Ignoring all points from Joint Trajectory except the final point");
    }

    const trajectory_msgs::JointTrajectoryPoint& final_point = command_msg.points.back();
    if (final_point.positions.size() != joint_names().size()) {
        ROS_ERROR("Insufficient joint position information");
        return {};
    }

    std::vector<double> target;
    target.reserve(joint_names().size());

    for (const std::string& joint_name : joint_names()) {
        int index = find_joint_index(joint_name, command_msg);
        if (index < 0) {
            ROS_ERROR("Failed to find position information for joint %s", joint_name.c_str());
            return {};
        }
        else {
            target.push_back(final_point.positions[index]);
        }
    }

    return target;
}

void ManipulatorInterfaceLiveROS::acknowledge_reset_callback(const hdt::AcknowledgeReset::ConstPtr& msg)
{
    ManipulatorError error = manip_.acknowledgeReset();
    ROS_INFO("Acknowledge reset responded with %s", to_string(error).c_str());
}

bool ManipulatorInterfaceLiveROS::assert_joint_limits(const ManipulatorParameters& manip_params)
{
    if (robot_model_.joint_names().size() != manip_params.getNumJoints()) {
        ROS_ERROR("Robot Model and HDT Manipulator Interface have different number of joints");
        return false;
    }

    auto almost_equals = [](double u, double v, double eps) { return fabs(u - v) <= fabs(eps); };

    for (std::size_t i = 0; i < robot_model_.min_limits().size(); ++i) {
        double urdf_min_limit = robot_model_.min_limits()[i];
        double urdf_max_limit = robot_model_.max_limits()[i];

        float hdt_min_limit;
        ManipulatorError error = manip_params.getMinPosition(i, &hdt_min_limit);

        if (error != ManipulatorError::NO_ERROR()) {
            ROS_ERROR("Failed to get min limit for joint %zd", i);
            return false;
        }

        if (!almost_equals(urdf_min_limit, (double)hdt_min_limit, 0.5 * M_PI / 180.0)) {
            ROS_ERROR("Min Limit does not match for joint %zd (urdf: %0.3f, hdt: %0.3f)", i, urdf_min_limit, (double)hdt_min_limit);
            return false;
        }

        float hdt_max_limit;
        error = manip_params.getMaxPosition(i, &hdt_max_limit);

        if (error != ManipulatorError::NO_ERROR()) {
            ROS_ERROR("Failed to get max limit for joint %zd", i);
            return false;
        }

        if (!almost_equals(urdf_max_limit, (double)hdt_max_limit, 0.5 * M_PI / 180.0)) {
            ROS_ERROR("Max Limit does not match for joint %zd (urdf: %0.3f, hdt: %0.3f)", i, urdf_max_limit, (double)hdt_max_limit);
            return false;
        }
    }

    return true;
}

} // namespace hdt
