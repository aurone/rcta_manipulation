// standard includes
#include <fstream>
#include <ros/ros.h>

// project includes
#include <config_block/config_block.h>

// module includes
#include "manipulator_interface_ros.h"

namespace hdt
{

std::string to_string(const ManipulatorError& error)
{
    if (error == ManipulatorError::NO_ERROR()) {
        return "NO_ERROR";
    }
    else if (error == ManipulatorError::POSITION_LIMIT_REACHED()) {
        return "POSITION LIMIT REACHED";
    }
    else if (error == ManipulatorError::POSITION_LIMIT_VIOLATION()) {
        return "POSITION LIMIT VIOLATION";
    }
    else if (error == ManipulatorError::VELOCITY_LIMIT_REACHED()) {
        return "VELOCITY LIMIT REACHED";
    }
    else if (error == ManipulatorError::VELOCITY_LIMIT_VIOLATION()) {
        return "VELOCITY LIMIT VIOLATION";
    }
    else if (error == ManipulatorError::ACCELERATION_LIMIT_REACHED()) {
        return "ACCELERATION LIMIT REACHED";
    }
    else if (error == ManipulatorError::ACCELERATION_LIMIT_VIOLATION()) {
        return "ACCELERATION LIMIT VIOLATION";
    }
    else if (error == ManipulatorError::TORQUE_LIMIT_REACHED()) {
        return "TORQUE LIMIT REACHED";
    }
    else if (error == ManipulatorError::TORQUE_LIMIT_VIOLATION()) {
        return "TORQUE LIMIT VIOLATION";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID()) {
        return "PARAM FILE INVALID";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_COUNT()) {
        return "PARAM FILE INVALID COUNT";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_JOINTS()) {
        return "PARAM FILE INVALID JOINTS";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_DH()) {
        return "PARAM FILE INVALID DH";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_POSLIMITS()) {
        return "PARAM FILE INVALID POS LIMITS";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_STOW()) {
        return "PARAM FILE INVALID STOW";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_VELOCITY()) {
        return "PARAM_FILE_INVALID_VELOCITY";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_ACCELERATION()) {
        return "PARAM_FILE_INVALID_ACCELERATION";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_RESOLUTION()) {
        return "PARAM_FILE_INVALID_RESOLUTION";
    }
    else if (error == ManipulatorError::PARAM_FILE_INVALID_ENC()) {
        return "PARAM FILE INVALID ENC";
    }
    else if (error == ManipulatorError::PARAM_FILE_NOT_FOUND()) {
        return "PARAM FILE NOT FOUND";
    }
    else if (error == ManipulatorError::PARAMETERS_NOT_INITIALIZED()) {
        return "PARAMETERS NOT INITIALIZED";
    }
    else if (error == ManipulatorError::PARAMETERS_NOT_KNOWN()) {
        return "PARAMETERS NOT KNOWN";
    }
    else if (error == ManipulatorError::INVALID_INDEX()) {
        return "INVALID INDEX";
    }
    else if (error == ManipulatorError::CALIBRATION_FAILURE()) {
        return "CALIBRATION FAILURE";
    }
    else if (error == ManipulatorError::JOINT_NOT_CALIBRATED()) {
        return "JOINT NOT CALIBRATED";
    }
    else if (error == ManipulatorError::NO_IMPLEMENTATION()) {
        return "NO IMPLEMENTATION";
    }
    else if (error == ManipulatorError::NO_COMMUNICATION()) {
        return "NO COMMUNICATION";
    }
    else if (error == ManipulatorError::NO_FEEDBACK()) {
        return "NO FEEDBACK";
    }
    else if (error == ManipulatorError::INTERFACE_NOT_INITIALIZED()) {
        return "INTERFACE NOT INITIALIZED";
    }
    else if (error == ManipulatorError::SET_VOLTAGE_ERROR()) {
        return "SET VOLTAGE ERROR";
    }
    else if (error == ManipulatorError::GET_VOLTAGE_ERROR()) {
        return "GET VOLTAGE ERROR";
    }
    else if (error == ManipulatorError::CONTROL_ERROR()) {
        return "CONTROL ERROR";
    }
    else if (error == ManipulatorError::RESET_ERROR()) {
        return "RESET ERROR";
    }
    else if (error == ManipulatorError::VALUE_OUT_OF_RANGE()) {
        return "VALUE OUT OF RANGE";
    }
    else {
        return "UNRECOGNIZED MANIPULATOR ERROR";
    }
}

const char *ManipulatorInterfaceROS::RunResultToString(RunResult result)
{
    switch (result)
    {
    case SUCCESS:
        return "SUCCESS";
    case FAILED_TO_INITIALIZE:
        return "FAILED_TO_INITIALIZE";
    default:
        return "UNKNOWN_RESULT";
    }
}

ManipulatorInterfaceROS::ManipulatorInterfaceROS() :
    joint_names_({"arm_1_shoulder_twist",
                  "arm_2_shoulder_lift",
                  "arm_3_elbow_twist",
                  "arm_4_elbow_lift",
                  "arm_5_wrist_twist",
                  "arm_6_wrist_lift",
                  "arm_7_gripper_lift"})
{
}

bool ManipulatorInterfaceROS::read_manip_params(const std::string& manip_config_fname, ManipulatorParameters& out)
{
    std::ifstream f(manip_config_fname.c_str(), std::ios::in | std::ios::binary);
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

    au::ConfigBlock config;
    if (!config.load(contents, "HDTManipulator")) {
        ROS_ERROR("Failed to load config for 'HDTManipulator' in %s", manip_config_fname.c_str());
        return false;
    }

    au::ConfigBlock joints_config;
    if (!config.get("joints", joints_config)) {
        ROS_ERROR("Failed to retrieve 'joints' config");
        return false;
    }

    ManipulatorParameters params;
    ROS_INFO("HDT Manipulator Joint Configuration:");
    for (const std::string& joint_name : joint_names()) {
        au::ConfigBlock joint_config;
        if (!joints_config.get(joint_name, joint_config)) {
            ROS_ERROR("Failed to retrieve '%s'", joint_name.c_str());
            return false;
        }

        ROS_INFO("    %s:", joint_name.c_str());
        double dh_angle, dh_length, dh_offset, dh_twist, resolution,
               min_position, max_position, max_vel, max_accel, stow_position;
        int joint_type; // TODO: examine the values for JointType.Revolute and JointType.Prismatic

        if (!joint_config.get("dh_angle", dh_angle) || !joint_config.get("dh_length", dh_length) ||
            !joint_config.get("dh_offset", dh_offset) || !joint_config.get("dh_twist", dh_twist) ||
            !joint_config.get("joint_type", joint_type) || !joint_config.get("resolution", resolution) ||
            !joint_config.get("min_position", min_position) || !joint_config.get("max_position", max_position) ||
            !joint_config.get("max_velocity", max_vel) || !joint_config.get("max_acceleration", max_accel) ||
            !joint_config.get("stow_position", stow_position))
        {
            ROS_ERROR("Failed to retrieve joint parameters");
            return false;
        }

        ROS_INFO("           dh_angle: %0.6f", dh_angle);
        ROS_INFO("          dh_length: %0.6f", dh_length);
        ROS_INFO("          dh_offset: %0.6f", dh_offset);
        ROS_INFO("           dh_twist: %0.6f", dh_twist);
        ROS_INFO("         joint_type: %s", joint_type == 0 ? "REVOLUTE" : "PRISMATIC");
        ROS_INFO("         resolution: %0.6f", resolution);
        ROS_INFO("       min_position: %0.6f", min_position);
        ROS_INFO("       max_position: %0.6f", max_position);
        ROS_INFO("       max_velocity: %0.6f", max_vel);
        ROS_INFO("          max_accel: %0.6f", max_accel);
        ROS_INFO("      stow position: %0.6f", stow_position);

        ManipulatorJointParameters m;
        ManipulatorJointType type = (joint_type == 0) ? ManipulatorJointType::Revolute() : ManipulatorJointType::Prismatic();
        m.DhAngle(dh_angle).DhLength(dh_length).DhOffset(dh_offset).DhTwist(dh_twist).Type(type);
        m.MinPosition(min_position).MaxPosition(max_position).MaxVelocity(max_vel).MaxAcceleration(max_accel);
        m.StowPosition(stow_position).Resolution(resolution);
        m.Name(joint_name);
        m.EncOffset(0.0f).EncScale(1.0);

        params.Append(m);
    }

    ROS_INFO("Manipulator Parameters contains information for %u joints", params.NumJoints());
    out = params;
    return true;
}

bool ManipulatorInterfaceROS::check_joints(const std::vector<std::string>& injoints) const
{
    if (injoints.size() != joint_names().size()) {
        return false;
    }

    for (size_t i = 0; i < injoints.size(); ++i) {
        if (count(joint_names().begin(), joint_names().end(), injoints[i]) != 1) {
            return false;
        }
    }

    for (size_t i = 0; i < joint_names().size(); ++i) {
        if (count(injoints.begin(), injoints.end(), joint_names()[i]) != 1) {
            return false;
        }
    }

    return true;
}

int ManipulatorInterfaceROS::find_joint_index(
    const std::string& joint_name,
    const trajectory_msgs::JointTrajectory& joint_traj) const
{
    for (size_t i = 0; i < joint_traj.joint_names.size(); ++i) {
        if (joint_traj.joint_names[i] == joint_name) {
            return i;
        }
    }
    return -1;
}

} // namespace hdt
