#ifndef stringifier_h
#define stringifier_h

#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/OrientedBoundingBox.h>

////////////////////////////////
/// Because, let's be honest ///
////////////////////////////////

template <typename T>
std::string to_string(const std::vector<T>& v);

std::string to_string(const std::vector<double>& v);

std::string to_string(const Eigen::Affine3d& transform);
std::string to_string(const Eigen::Vector2d& v);
std::string to_string(const Eigen::Vector3d& v);
std::string to_string(const Eigen::AngleAxisd& aa);

std::string to_string(const geometry_msgs::Quaternion& quat);
std::string to_string(const geometry_msgs::Point& point);
std::string to_string(const geometry_msgs::Point32& point);
std::string to_string(const geometry_msgs::Pose& pose);
std::string to_string(const moveit_msgs::OrientedBoundingBox& bbx);

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
    ss << "pos: (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rot: (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

inline std::string to_string(const Eigen::Vector2d& v)
{
    std::stringstream ss;
    ss << "(" << v(0) << ", " << v(1) << ")";
    return ss.str();
}

inline std::string to_string(const Eigen::Vector3d& v)
{
    std::stringstream ss;
    ss << "(" << v(0) << ", " << v(1) << ", " << v(2) << ")";
    return ss.str();
}

inline std::string to_string(const Eigen::AngleAxisd& aa)
{
    std::stringstream ss;
    ss << "{ angle: " << sbpl::utils::ToDegrees(aa.angle()) << " degs @ " << to_string(aa.axis()) << " }";
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

inline std::string to_string(const geometry_msgs::Quaternion& quat)
{
    std::stringstream ss;
    ss << "(" << quat.w << ", " << quat.x << ", " << quat.y << ", " << quat.z << ")";
    return ss.str();
}

inline std::string to_string(const geometry_msgs::Point& point)
{
    std::stringstream ss;
    ss << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return ss.str();
}

inline std::string to_string(const geometry_msgs::Point32& point)
{
    std::stringstream ss;
    ss << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return ss.str();
}

inline std::string to_string(const geometry_msgs::Pose& pose)
{
    std::stringstream ss;
    ss << "{ position: " << to_string(pose.position) << ", " << "orientation: " << to_string(pose.orientation) << " }";
    return ss.str();
}

inline std::string to_string(const moveit_msgs::OrientedBoundingBox& bbx)
{
    std::stringstream ss;
    ss << "{ pose: " << to_string(bbx.pose) << ", extents: " << to_string(bbx.extents) << " }";
    return ss.str();
}

inline const char* visualization_msgs_marker_type_to_cstr(int32_t type)
{
    switch (type)
    {
    case visualization_msgs::Marker::ARROW:
        return "Arrow";
    case visualization_msgs::Marker::CUBE:
        return "Cube";
    case visualization_msgs::Marker::SPHERE:
        return "Sphere";
    case visualization_msgs::Marker::CYLINDER:
        return "Cylinder";
    case visualization_msgs::Marker::LINE_STRIP:
        return "Line Strip";
    case visualization_msgs::Marker::LINE_LIST:
        return "Line List";
    case visualization_msgs::Marker::CUBE_LIST:
        return "Cube List";
    case visualization_msgs::Marker::SPHERE_LIST:
        return "Sphere List";
    case visualization_msgs::Marker::POINTS:
        return "Points";
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        return "Text View Facing";
    case visualization_msgs::Marker::MESH_RESOURCE :
        return "Mesh Resource";
    case visualization_msgs::Marker::TRIANGLE_LIST :
        return "Triangle List";
    default:
        return "Unknown";
    }
}

#endif
