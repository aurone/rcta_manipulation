#include "msg_utils.h"

#include <Eigen/Dense>
#include <algorithm>
#include <sbpl_geometry_utils/utils.h>
#include <tf/transform_datatypes.h>

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

visualization_msgs::Marker create_arrow_marker(const geometry_msgs::Vector3 &scale)
{
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.seq = 0;
    arrow_marker.header.stamp = ros::Time(0);
    arrow_marker.header.frame_id = "";
    arrow_marker.ns = "";
    arrow_marker.id = 0;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.position.x = 0.0;
    arrow_marker.pose.position.y = 0.0;
    arrow_marker.pose.position.z = 0.0;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.pose.orientation.x = 0.0;
    arrow_marker.pose.orientation.y = 0.0;
    arrow_marker.pose.orientation.z = 0.0;
    arrow_marker.scale.x = scale.x;
    arrow_marker.scale.y = scale.y;
    arrow_marker.scale.z = scale.z;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.7;
    arrow_marker.lifetime = ros::Duration(0);
    arrow_marker.frame_locked = false;
    return arrow_marker;
}

visualization_msgs::MarkerArray create_triad_marker_arr(const geometry_msgs::Vector3& scale)
{
    visualization_msgs::MarkerArray markers;

    // create an arrow marker for the x-axis
    visualization_msgs::Marker m = create_arrow_marker(scale);
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.7;
    m.id = 0;
    markers.markers.push_back(m);

    // create an arrow marker for the y-axis
    Eigen::AngleAxisd rotate_z(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Quaterniond q = rotate_z * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.id = 1;
    markers.markers.push_back(m);

    // create an arrow marker for the z-axis
    Eigen::AngleAxisd rotate_y(-M_PI / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0));
    q = rotate_y * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.id = 2;
    markers.markers.push_back(m);

    visualization_msgs::Marker origin_marker;
    origin_marker.header.seq = 0;
    origin_marker.header.stamp = ros::Time(0);
    origin_marker.header.frame_id = "";
    origin_marker.ns = "";
    origin_marker.id = 0;
    origin_marker.type = visualization_msgs::Marker::CUBE;
    origin_marker.action = visualization_msgs::Marker::ADD;
    origin_marker.pose.position.x = origin_marker.pose.position.y = origin_marker.pose.position.z = 0.0;
    origin_marker.pose.orientation.w = 1.0;
    origin_marker.pose.orientation.x = origin_marker.pose.orientation.y = origin_marker.pose.orientation.z = 0.0;
    origin_marker.scale.x = origin_marker.scale.y = origin_marker.scale.z = scale.y; // note: assume that scale.y and scale.z are the same
    origin_marker.color.r = origin_marker.color.g = origin_marker.color.b = 0.9;
    origin_marker.color.a = 1.0;
    origin_marker.lifetime = ros::Duration(0);
    origin_marker.frame_locked = false;
    origin_marker.id = 3;
    markers.markers.push_back(origin_marker);

    return markers;
}

Eigen::Affine3d interp(const Eigen::Affine3d& s, const Eigen::Affine3d& t, double alpha)
{
    Eigen::Vector3d interp_pos = (1.0 - alpha) * Eigen::Vector3d(s.translation()) +
                                        alpha  * Eigen::Vector3d(t.translation());
    Eigen::Quaterniond aq(s.rotation());
    Eigen::Quaterniond bq(t.rotation());
    Eigen::Quaterniond interp_rot = aq.slerp(alpha, bq);
    return Eigen::Translation3d(interp_pos) * interp_rot;
}

Eigen::Affine3d transform_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
{
    Eigen::Vector3d apos(a.translation());
    Eigen::Vector3d bpos(b.translation());

    Eigen::Quaterniond arot(a.rotation());
    Eigen::Quaterniond brot(b.rotation());

    return Eigen::Affine3d(Eigen::Translation3d(apos - bpos) * Eigen::Quaterniond(arot.inverse() * brot));
}

void get_euler_ypr(const Eigen::Affine3d& transform, double& yaw, double& pitch, double& roll)
{
    Eigen::Quaterniond q(transform.rotation());
    Eigen::Vector3d p(transform.translation());
    tf::Transform goalTF(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(p.x(), p.y(), p.z()));
    goalTF.getBasis().getEulerYPR(yaw, pitch, roll, 1);
}

const char* to_string(XmlRpc::XmlRpcValue::Type type)
{
    switch (type) {
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Integer";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:
        return "Base64";
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, bool& bout)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        ROS_WARN("Expected XML value of type boolean (got %s)", to_string(value.getType()));
        return false;
    }
    else {
        bout = bool(value);
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, int& iout)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_WARN("Expected XML value of type integer (got %s)", to_string(value.getType()));
        return false;
    }
    else {
        iout = int(value);
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, double& dout)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        ROS_WARN("Expected XML value of type double (got %s)", to_string(value.getType()));
        return false;
    }
    else {
        dout = double(value);
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, std::string& sout)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Expected XML value of type string (got %s)", to_string(value.getType()));
        return false;
    }
    else {
        sout = std::string(value);
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, geometry_msgs::Point& p)
{
    std::map<std::string, double> params;
    bool success = extract_xml_value(value, params);
    if (!success ||
        params.find("x") == params.end() ||
        params.find("y") == params.end() ||
        params.find("z") == params.end())
    {
        return false;
    }
    else {
        p.x = params["x"];
        p.y = params["y"];
        p.z = params["z"];
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, geometry_msgs::Quaternion& q)
{
    std::map<std::string, double> params;
    if (!extract_xml_value(value, params) ||
        params.find("w") == params.end() ||
        params.find("x") == params.end() ||
        params.find("y") == params.end() ||
        params.find("z") == params.end())
    {
        return false;
    }
    else
    {
        q.w = params["w"];
        q.x = params["x"];
        q.y = params["y"];
        q.z = params["z"];
        return true;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, geometry_msgs::Pose& p)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        return false;
    }

    if (!value.hasMember("position") || !value.hasMember("orientation")) {
        return false;
    }
    else {
        geometry_msgs::Pose tmp;
        bool success = extract_xml_value(value["position"], tmp.position) &&
                       extract_xml_value(value["orientation"], tmp.orientation);
        if (success) {
            p = tmp;
            return true;
        }
        else {
            return false;
        }
    }
}

std::vector<double> to_degrees(const std::vector<double>& v)
{
    std::vector<double> v_degs(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        v_degs[i] = sbpl::utils::ToDegrees(v[i]);
    }
    return v_degs;
}

std::vector<double> to_radians(const std::vector<double>& v)
{
    std::vector<double> v_rads(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        v_rads[i] = sbpl::utils::ToRadians(v[i]);
    }
    return v_rads;
}

void convert(const tf::Transform& from, Eigen::Affine3d& to)
{
    const double tx = from.getOrigin().x();
    const double ty = from.getOrigin().y();
    const double tz = from.getOrigin().z();

    double qw = from.getRotation().w();
    double qx = from.getRotation().x();
    double qy = from.getRotation().y();
    double qz = from.getRotation().z();

    to = Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);
}

} // namespace msg_utils

namespace geometry_msgs
{

Vector3 CreateVector3(double x, double y, double z)
{
    Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

const Vector3 ZeroVector3()
{
    return CreateVector3(0.0, 0.0, 0.0);
}

Point CreatePoint(double x, double y, double z)
{
    Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

const Point ZeroPoint()
{
    return CreatePoint(0.0, 0.0, 0.0);
}

Quaternion CreateQuaternion(double w, double x, double y, double z)
{
    Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

const Quaternion IdentityQuaternion()
{
    return CreateQuaternion(1.0, 0.0, 0.0, 0.0);
}

Pose CreatePose(const Point& position, const Quaternion& orientation)
{
    Pose p;
    p.position = position;
    p.orientation = orientation;
    return p;
}

const Pose IdentityPose()
{
    return CreatePose(ZeroPoint(), IdentityQuaternion());
}

} // namespace geometry_msgs

namespace std_msgs
{

ColorRGBA CreateColorRGBA(float r, float g, float b, float a)
{
    ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

const ColorRGBA BlackColorRGBA(float a)
{
    return CreateColorRGBA(0.0f, 0.0f, 0.0f, a);
}

const ColorRGBA RedColorRGBA(float a)
{
    return CreateColorRGBA(1.0f, 0.0f, 0.0f, a);
}

const ColorRGBA GreenColorRGBA(float a)
{
    return CreateColorRGBA(0.0f, 1.0f, 0.0f, a);
}

const ColorRGBA BlueColorRGBA(float a)
{
    return CreateColorRGBA(0.0f, 0.0f, 1.0f, a);
}

const ColorRGBA YellowColorRGBA(float a)
{
    return CreateColorRGBA(1.0f, 1.0f, 0.0f, a);
}

const ColorRGBA CyanColorRGBA(float a)
{
    return CreateColorRGBA(0.0f, 1.0f, 1.0f, a);
}

const ColorRGBA MagentaColorRGBA(float a)
{
    return CreateColorRGBA(1.0f, 0.0f, 1.0f, a);
}

const ColorRGBA WhiteColorRGBA(float a)
{
    return CreateColorRGBA(1.0f, 1.0f, 1.0f, a);
}

}
