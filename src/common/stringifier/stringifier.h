#ifndef stringifier_h
#define stringifier_h

#include <array>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

////////////////////////////////
/// Because, let's be honest ///
////////////////////////////////

std::string to_string(const Eigen::Affine3d& transform);
std::string to_string(const std::array<double, 7>& joint_state);
std::string to_string(const geometry_msgs::Pose& pose);

template <typename T>
std::string to_string(const std::vector<T>& v)
{
    std::stringstream ss;
    ss << "[ ";
    for (const T& t : v) {
        ss << t << ' ';
    }
    ss << ']';
    return ss.str();
}

std::string to_string(const Eigen::Affine3d& transform)
{
    const Eigen::Vector3d translation(transform.translation());
    const Eigen::Quaterniond rotation(transform.rotation());
    std::stringstream ss;
    ss << "translation(x, y, z): (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rotation(w, x, y, z): (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

std::string to_string(const std::array<double, 7>& joint_state)
{
    std::stringstream ss;
    ss << std::setprecision(3) << std::setw(8) << '(' <<
          joint_state[0] << ' ' <<
          joint_state[1] << ' ' <<
          joint_state[2] << ' ' <<
          joint_state[3] << ' ' <<
          joint_state[4] << ' ' <<
          joint_state[5] << ' ' <<
          joint_state[6] << ')';
    return ss.str();
}

std::string to_string(const geometry_msgs::Pose& pose)
{
    std::stringstream ss;
    ss << "{ position: {";
    ss << "x: " << pose.position.x << ", ";
    ss << "y: " << pose.position.y << ", ";
    ss << "z: " << pose.position.z << " } ";
    ss << "orientation: {";
    ss << "w: " << pose.orientation.w << ", ";
    ss << "x: " << pose.orientation.x << ", ";
    ss << "y: " << pose.orientation.y << ", ";
    ss << "z: " << pose.orientation.z << "} }";
    return ss.str();
}

#endif
