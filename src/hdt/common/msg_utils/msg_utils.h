#ifndef msg_utils_h
#define msg_utils_h

#include <string>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace msg_utils
{

/// @brief Return whether two vectors contain the same set of elements.
template <typename T>
bool vector_sets_equal(const std::vector<T>& u, const std::vector<T>& v);

/// @brief Return whether a joint state contains a given set of joints
bool contains_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints);

/// @brief Return whether a joint state contains exactly a given set of joints
bool contains_only_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints);

/// @brief Reorder the joints in a joint state message.
/// @return false if the joint state contains joints other than those specified in the joint order; true otherwise
bool reorder_joints(sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_order);

bool reorder_joints(trajectory_msgs::JointTrajectory& joint_trajectory, const std::vector<std::string>& joint_order);

visualization_msgs::Marker create_arrow_marker(const geometry_msgs::Vector3 &scale);
visualization_msgs::MarkerArray create_triad_marker_arr(const geometry_msgs::Vector3& scale);

////////////////////////////////////////////////////////////////////////////////
// Template Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
bool vector_sets_equal(const std::vector<T>& u, const std::vector<T>& v)
{
    if (u.size() != v.size()) {
        return false;
    }

    for (size_t i = 0; i < u.size(); ++i) {
        if (std::count(v.begin(), v.end(), u[i]) != 1) {
            return false;
        }
    }

    for (size_t i = 0; i < v.size(); ++i) {
        if (std::count(u.begin(), u.end(), v[i]) != 1) {
            return false;
        }
    }

    return true;
}

} // namespace msg_utils

#endif
