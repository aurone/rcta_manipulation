#include "msg_utils.h"

#include <algorithm>

namespace msg_utils
{

bool contains_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints)
{
    for (const std::string& joint_name : joints) {
        if (std::find(joint_state.name.begin(), joint_state.name.end(), joint_name) == joint_state.name.end()) {
            return false;
        }
    }
    return true;
}

bool contains_only_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints)
{
    return vector_sets_equal(joint_state.name, joints);
}

bool reorder_joints(sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_order)
{
    if (!contains_only_joints(joint_state, joint_order)) {
        return false;
    }

    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        const std::string& jo_joint_name = joint_order[i];
        for (std::size_t j = i; j < joint_state.name.size(); ++j) {
            std::string& js_joint_name = joint_state.name[j];
            if (js_joint_name == jo_joint_name && i != j) {
                swap(joint_state.name[i], joint_state.name[j]);
                std::swap(joint_state.position[i], joint_state.position[j]);
                std::swap(joint_state.velocity[i], joint_state.velocity[j]);
                std::swap(joint_state.effort[i], joint_state.effort[j]);
            }
        }
    }

    return true;
}

bool reorder_joints(trajectory_msgs::JointTrajectory& joint_trajectory, const std::vector<std::string>& joint_order)
{
    if (!vector_sets_equal(joint_trajectory.joint_names, joint_order)) {
        return false;
    }

    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        const std::string& jo_joint_name = joint_order[i];
        for (std::size_t j = i; j < joint_trajectory.joint_names.size(); ++j) {
            std::string& js_joint_name = joint_trajectory.joint_names[j];
            if (js_joint_name == jo_joint_name && i != j) {
                swap(joint_trajectory.joint_names[i], joint_trajectory.joint_names[j]);
                for (trajectory_msgs::JointTrajectoryPoint point : joint_trajectory.points) {
                    if (!point.positions.empty()) {
                        std::swap(point.positions[i], point.positions[j]);
                    }
                    if (!point.velocities.empty()) {
                        std::swap(point.velocities[i], point.velocities[j]);
                    }
                    if (!point.accelerations.empty()) {
                        std::swap(point.accelerations[i], point.accelerations[j]);
                    }
                }
            }
        }
    }

    return true;
}

} // namespace msg_utils
