#ifndef stringifier_h
#define stringifier_h

#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

////////////////////////////////
/// Because, let's be honest ///
////////////////////////////////

template <typename T>
std::string to_string(const std::vector<T>& v);

std::string to_string(const std::vector<double>& v);

std::string to_string(const Eigen::Affine3d& transform);
std::string to_string(const geometry_msgs::Pose& pose);

template <typename T, std::size_t N>
std::string to_string(const std::array<T, N>& arr);

template <std::size_t N>
std::string to_string(const std::array<double, N>& arr);

//////////////////////////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
inline std::string to_string(const std::vector<T>& v)
{
    std::stringstream ss;
    ss << "[ ";
    for (const T& t : v) {
        ss << t << ' ';
    }
    ss << ']';
    return ss.str();
}

inline std::string to_string(const std::vector<double>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (int i = 0; i < (int)vec.size(); ++i) {
        // output doubles in fixed-point notation with 3 digits after the
        // decimal point; allow space for the decimal point, leading 0, and
        // possible - sign
        ss << std::fixed << std::setprecision(3) << std::setw(6) << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << " ]";
    return ss.str();
}

inline std::string to_string(const Eigen::Affine3d& transform)
{
    const Eigen::Vector3d translation(transform.translation());
    const Eigen::Quaterniond rotation(transform.rotation());
    std::stringstream ss;
    ss << "translation(x, y, z): (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rotation(w, x, y, z): (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

template <class T, std::size_t N>
inline std::string to_string(const std::array<T, N>& arr)
{
    std::stringstream ss;
    ss << "( ";
    for (std::size_t i = 0; i < N; ++i) {
        ss << arr[i] << ' ';
    }
    ss << ')';
    return ss.str();

}

template <std::size_t N>
inline std::string to_string(const std::array<double, N>& arr)
{
    std::stringstream ss;
    ss << std::setprecision(3) << std::setw(8);
    ss << "( ";
    for (std::size_t i = 0; i < N; ++i) {
        ss << arr[i] << ' ';
    }
    ss << ')';
    return ss.str();
}

inline std::string to_string(const geometry_msgs::Pose& pose)
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
