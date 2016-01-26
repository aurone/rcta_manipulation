#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <hdt/TeleopDiagnosticStatus.h>
#include <hdt/AcknowledgeReset.h>
#include <config_block/config_block.h>
#include "teleop_node.h"
#include "controller.h"

namespace hdt
{

const double TeleopNode::MAX_DELTA_POS_RPS = 180.0 * M_PI / 180.0;
const double TeleopNode::MAX_DELTA_VEL_RPSPS = 20.0;
const double TeleopNode::MAX_DELTA_TORQUE_NMPS = 20.0;
const double TeleopNode::MAX_DELTA_CURRENT_APS = 20.0;

TeleopNode::TeleopNode() :
    joystick_(),
    nh_(),
    ph_("~"),
    hdt_diagnostic_sub_(),
    joint_state_sub_(),
    command_pub_(),
    button_event_handlers_(),
    axis_event_handlers_(),
    shutdown_(false),
    loop_rate_hz_(0.0),
    controller_(),
    joint_names_({"arm_1_shoulder_twist", "arm_2_shoulder_lift", "arm_3_elbow_twist", "arm_4_elbow_lift", "arm_5_wrist_twist", "arm_6_wrist_lift", "arm_7_gripper_lift"}),
    joint_positions_(joint_names_.size(), 0.0),
    joint_velocities_(joint_names_.size(), 0.0),
    selected_joint_idx_(0),
    joint_pos_target_(joint_names_.size(), 0.0),
    joint_vel_target_(joint_names_.size(), 20.0 * M_PI / 180.0),
    last_joint_state_(),
    deadman_on_(false)
{
}

TeleopNode::~TeleopNode()
{
}

TeleopNode::RunResult TeleopNode::run()
{
    if (!init()) {
        ROS_ERROR("Failed to initialize Teleop Node");
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(loop_rate_hz_);
    while (ros::ok() && !shutdown_) {
        ros::spinOnce();
        controller_->process_events();

        // Don't move the arm until we've heard from it
        if (last_joint_state_ && deadman_on_) {
            trajectory_msgs::JointTrajectory joint_trajectory;
            static int seqno = 0;
            joint_trajectory.header.seq = seqno++;
            joint_trajectory.header.stamp = ros::Time::now();
            joint_trajectory.header.frame_id = "";
            joint_trajectory.joint_names = joint_names_;
            joint_trajectory.points.resize(1);
            joint_trajectory.points[0].positions = joint_pos_target_;
            joint_trajectory.points[0].velocities = joint_vel_target_;
            joint_trajectory.points[0].accelerations = std::vector<double>(joint_names_.size(), 0.0); //joint_accel_target_;
            command_pub_.publish(joint_trajectory);
        }

        hdt::TeleopDiagnosticStatus status;
        status.name     = joint_names_;
        status.position = joint_pos_target_;
        status.velocity = joint_vel_target_;
        status.effort   = std::vector<double>(joint_names_.size(), 0.0);

        status.at_limit = hdt::TeleopDiagnosticStatus::_at_limit_type(joint_names_.size(), false);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            status.at_limit[i] = at_limit(i);
        }

        status.current_selection = selected_joint_idx_;
        status.deadman           = deadman_on_;
        diagnostic_pub_.publish(status);

        loop_rate.sleep();
    }

    return SUCCESS;
}

bool TeleopNode::init()
{
    std::string urdfstr;
    if (!nh_.getParam("robot_description", urdfstr)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from param server");
        return false;
    }

    robot_model_ = urdf::parseURDF(urdfstr);
    if (!robot_model_) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    joint_min_positions_.resize(joint_names_.size());
    joint_max_positions_.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& joint_name = joint_names_[i];
        boost::shared_ptr<const urdf::Joint> joint = robot_model_->getJoint(joint_name);
        if (!joint) {
            ROS_ERROR("Failed to find joint '%s' in robot model", joint_name.c_str());
            return false;
        }

        if (!joint->limits) {
            ROS_ERROR("Failed to find joint limits for joint '%s'", joint_name.c_str());
            return false;
        }

        joint_min_positions_[i] = joint->limits->lower;
        joint_max_positions_[i] = joint->limits->upper;
    }

    std::string mappings_fname;
    if (!ph_.getParam("mappings_fname", mappings_fname)) {
        ROS_ERROR("Failed to retrieve 'mappings_fname' from param server");
        return false;
    }

    std::string joystick_path;
    if (!ph_.getParam("joystick_path", joystick_path)) {
        ROS_ERROR("Failed to retrieve 'joystick_path' from param server");
        return false;
    }

    ph_.param("loop_rate", loop_rate_hz_, 60.0);
    ROS_WARN("Running at %0.3f Hz", loop_rate_hz_);

    // read in key mappings file
    std::ifstream f(mappings_fname.c_str(), std::ios::in | std::ios::binary);
    std::string contents;
    if (f) {
        f.seekg(0, std::ios::end);
        contents.resize(f.tellg());
        f.seekg(0, std::ios::beg);
        f.read(&contents[0], contents.size());
        f.close();
    }
    else {
        return false;
    }

    // set up command index -> function handler mapping
    au::ConfigBlock command_config;
    if (!command_config.load(contents, "ButtonCommand")) {
        ROS_ERROR("Failed to load 'ButtonCommand' config");
        return false;
    }

    (void)register_button_callback(command_config, "NoCommand", &TeleopNode::no_command);
    (void)register_button_callback(command_config, "Quit", &TeleopNode::quit);
    (void)register_button_callback(command_config, "IncreaseCurrentJoint", &TeleopNode::increase_current_joint);
    (void)register_button_callback(command_config, "DecreaseCurrentJoint", &TeleopNode::decrease_current_joint);
    (void)register_button_callback(command_config, "ResetTargetPositionAndVelocity", &TeleopNode::reset_target_pos_and_vel);
    (void)register_button_callback(command_config, "ResetTargetTorqueAndCurrent", &TeleopNode::reset_target_torque_and_current);
    (void)register_button_callback(command_config, "SendMotionCommand", &TeleopNode::send_motion_command);
    (void)register_button_callback(command_config, "AcknowledgeReset", &TeleopNode::acknowledge_reset);
    (void)register_button_callback(command_config, "SendImpedanceCommand", &TeleopNode::send_impedance_command);
    (void)register_button_callback(command_config, "SendImpedanceOffCommand", &TeleopNode::send_impedance_off_command);
    (void)register_button_callback(command_config, "ResetImpedanceParameters", &TeleopNode::reset_impedance_params);
    (void)register_button_callback(command_config, "DeadManSwitchOn", &TeleopNode::dead_man_switch_on);
    (void)register_button_callback(command_config, "DeadManSwitchOff", &TeleopNode::dead_man_switch_off);

    au::ConfigBlock axis_command_config;
    if (!axis_command_config.load(contents, "AxisCommand")) {
        ROS_ERROR("Failed to load 'AxisCommand' config");
        return false;
    }

    (void)register_axis_callback(axis_command_config, "NoCommand", &TeleopNode::no_command);
    (void)register_axis_callback(axis_command_config, "Position", &TeleopNode::set_position);
    (void)register_axis_callback(axis_command_config, "Velocity", &TeleopNode::set_velocity);
    (void)register_axis_callback(axis_command_config, "Torque", &TeleopNode::set_torque);
    (void)register_axis_callback(axis_command_config, "MotorCurrent", &TeleopNode::set_current);
    (void)register_axis_callback(axis_command_config, "Inertia", &TeleopNode::set_inertia);
    (void)register_axis_callback(axis_command_config, "Damping", &TeleopNode::set_damping);
    (void)register_axis_callback(axis_command_config, "Stiffness", &TeleopNode::set_stiffness);

    (void)register_axis_callback(axis_command_config, "DecreasePosition", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "IncreasePosition", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "IncreaseVelocity", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "DecreaseVelocity", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "IncreaseTorque", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "DecreaseTorque", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "IncreaseMotorCurrent", &TeleopNode::some_fun);
    (void)register_axis_callback(axis_command_config, "DecreaseMotorCurrent", &TeleopNode::some_fun);

    au::ConfigBlock mappings_config;
    if (!mappings_config.load(contents, "Mappings")) {
        ROS_ERROR("Failed to load 'Mappings' config");
        return false;
    }

    controller_.reset(new Controller);
    if (!controller_) {
        return false;
    }

    if (!controller_->init(joystick_path, mappings_config, button_event_handlers_, axis_event_handlers_)) {
        return FAILED_TO_INITIALIZE;
    }

    joint_state_sub_ = nh_.subscribe("joint_states", 5, &TeleopNode::joint_state_callback, this);
    command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 5);
    diagnostic_pub_ = nh_.advertise<hdt::TeleopDiagnosticStatus>("teleop_diagnostics", 5);
    ack_reset_pub_ = nh_.advertise<hdt::AcknowledgeReset>("acknowledge_hdt_reset", 1);

    ROS_INFO("Initialized controller");
    return true;
}

bool TeleopNode::register_button_callback(
    const au::ConfigBlock& command_config,
    const std::string& command_name,
    ButtonCommandMemberFn method)
{
    int button_cmd_index;
    if (const_cast<au::ConfigBlock&>(command_config).get(command_name, button_cmd_index)) {
        this->button_event_handlers_[button_cmd_index] = [this, method]() { (this->*method)(); };
        return true;
    }
    else {
        ROS_WARN("Failed to find Button Command '%s'", command_name.c_str());
        return false;
    }
}

bool TeleopNode::register_axis_callback(
    const au::ConfigBlock& command_config,
    const std::string& command_name,
    AxisCommandMemberFn method)
{
    int axis_cmd_index;
    if (const_cast<au::ConfigBlock&>(command_config).get(command_name, axis_cmd_index)) {
        this->axis_event_handlers_[axis_cmd_index] = [this, method](double val) { (this->*method)(val); };
        return true;
    }
    else {
        ROS_WARN("Failed to find Axis Command '%s'", command_name.c_str());
    }
    return false;
}

void TeleopNode::shutdown()
{
    shutdown_ = true;
}

void TeleopNode::no_command(double value)
{
}

void TeleopNode::set_position(double value)
{
    if (!last_joint_state_) {
        return;
    }

    joint_pos_target_[selected_joint_idx_] = joint_positions_[selected_joint_idx_] + value * (MAX_DELTA_POS_RPS * (1.0 / loop_rate_hz_));

    // NOTE: check joint limits with leq/geq so that we can report "at limits" diagnostic information

//    if (joint_pos_target_[selected_joint_idx_] <= joint_min_positions_[selected_joint_idx_] + 0.01) {
//        ROS_WARN("Joint at min limit");
//        joint_pos_target_[selected_joint_idx_] = joint_min_positions_[selected_joint_idx_] + 0.01;
//    }
//
//    if (joint_pos_target_[selected_joint_idx_] >= joint_max_positions_[selected_joint_idx_] - 0.01) {
//        ROS_WARN("Joint at max limit");
//        joint_pos_target_[selected_joint_idx_] = joint_max_positions_[selected_joint_idx_] - 0.01;
//    }
}

void TeleopNode::set_velocity(double value)
{
//    if (!last_joint_state_) {
//        return;
//    }
//
//    joint_vel_target_[selected_joint_idx_] = joint_velocities_[selected_joint_idx_] + value * (MAX_DELTA_VEL_RPSPS * (1.0 / loop_rate_hz_));
}

void TeleopNode::set_torque(double value)
{
}

void TeleopNode::set_current(double value)
{
}

void TeleopNode::set_inertia(double value)
{

}

void TeleopNode::set_damping(double value)
{

}

void TeleopNode::set_stiffness(double value)
{

}

void TeleopNode::no_command()
{
}

void TeleopNode::quit()
{
    shutdown_ = true;
}

void TeleopNode::increase_current_joint()
{
    selected_joint_idx_ = (selected_joint_idx_ + 1) % joint_names_.size();
}

void TeleopNode::decrease_current_joint()
{
    selected_joint_idx_ = (selected_joint_idx_ + joint_names_.size() - 1) % joint_names_.size();
}

void TeleopNode::reset_target_pos_and_vel()
{
}

void TeleopNode::reset_target_torque_and_current()
{
}

void TeleopNode::send_motion_command()
{
}

void TeleopNode::acknowledge_reset()
{
    hdt::AcknowledgeReset reset_msg;
    ack_reset_pub_.publish(reset_msg);
}

void TeleopNode::send_impedance_command()
{
}

void TeleopNode::send_impedance_off_command()
{
}

void TeleopNode::reset_impedance_params()
{
}

void TeleopNode::dead_man_switch_on()
{
    deadman_on_ = true;
}

void TeleopNode::dead_man_switch_off()
{
    deadman_on_ = false;
}

void TeleopNode::some_fun(double)
{
}

void TeleopNode::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!last_joint_state_) {
        // seed joint positions
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            (void)get_joint_value(*msg, joint_names_[i], joint_pos_target_[i]);
            // (void)get_joint_velocity(*msg, joint_names_[i], joint_vel_target_[i]);
        }
    }

    last_joint_state_ = msg;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        (void)get_joint_value(*msg, joint_names_[i], joint_positions_[i]);
        // (void)get_joint_velocity(*msg, joint_names_[i], joint_velocities_[i]);
    }
}

bool TeleopNode::get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value) const
{
    if (joint_state.name.size() != joint_state.position.size()) {
        ROS_ERROR("Number of joint names and joint positions differ");
        return false;
    }

    for (int i = 0; i < (int)joint_state.name.size(); ++i) {
        if (joint_state.name[i] == joint) {
            joint_value = joint_state.position[i];
            return true;
        }
    }

    return false;
}

bool TeleopNode::get_joint_velocity(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_velocity) const
{
    if (joint_state.name.size() != joint_state.velocity.size()) {
        ROS_ERROR("Number of joint names and joint velocities differ");
        return false;
    }

    for (int i = 0; i < (int)joint_state.name.size(); ++i) {
        if (joint_state.name[i] == joint) {
            joint_velocity = joint_state.velocity[i];
            return true;
        }
    }

    return false;
}

bool TeleopNode::at_limit(int joint_index) const
{
    assert(joint_index >= 0 && joint_index < (int)joint_names_.size());

    return (joint_pos_target_[joint_index] <= joint_min_positions_[joint_index]) ||
           (joint_pos_target_[joint_index] >= joint_max_positions_[joint_index]);
}

} // namespace hdt
