#include "WorldSimulatorNode.h"

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <moveit_msgs/CollisionObject.h>
#include <hdt/common/msg_utils/msg_utils.h>

bool extract_xml_value(XmlRpc::XmlRpcValue& value, EulerAngles& angles)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        return false;
    }

    bool success =
            msg_utils::extract_xml_value(value["r"], angles.r) &&
            msg_utils::extract_xml_value(value["p"], angles.p) &&
            msg_utils::extract_xml_value(value["y"], angles.y);

    return success;
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, SixDofPose& pose)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        return false;
    }

    bool success =
            msg_utils::extract_xml_value(value["position"], pose.pos) &&
            ::extract_xml_value(value["orientation"], pose.rot);

    return success;
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, SimpleCollisionObject& cobject)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        return false;
    }

    bool success =
            msg_utils::extract_xml_value(value["id"], cobject.id) &&
            msg_utils::extract_xml_value(value["type"], cobject.type) &&
            ::extract_xml_value(value["pose"], cobject.pose);

    if (!success) {
        return false;
    }

    std::map<std::string, double> dims;
    if (!msg_utils::extract_xml_value(value["dims"], dims)) {
        ROS_WARN("Failed to retrieve collision object shape dimensions");
        return false;
    }

    // check for existence of appropriate dimensions
    if (cobject.type == "box") {
        success =
                dims.find("length") != dims.end() &&
                dims.find("width") != dims.end() &&
                dims.find("height") != dims.end();
        if (success) {
            cobject.dims[SimpleCollisionObject::BOX_LENGTH] = dims["length"];
            cobject.dims[SimpleCollisionObject::BOX_WIDTH ] = dims["width"];
            cobject.dims[SimpleCollisionObject::BOX_HEIGHT] = dims["height"];
        }
    }
    else if (cobject.type == "sphere") {
        success = dims.find("radius") != dims.end();
        if (success) {
            cobject.dims[SimpleCollisionObject::SPHERE_RADIUS] = dims["radius"];
        }
    }
    else if (cobject.type == "cylinder") {
        success = dims.find("height") != dims.end() && dims.find("radius") != dims.end();
        if (success) {
            cobject.dims[SimpleCollisionObject::CYLINDER_HEIGHT] = dims["height"];
            cobject.dims[SimpleCollisionObject::CYLINDER_RADIUS ] = dims["radius"];
        }
    }
    else if (cobject.type == "cone") {
        success = dims.find("height") != dims.end() && dims.find("radius") != dims.end();
        if (success) {
            cobject.dims[SimpleCollisionObject::CONE_HEIGHT] = dims["height"];
            cobject.dims[SimpleCollisionObject::CONE_RADIUS ] = dims["radius"];
        }
    }
    else {
        ROS_WARN("Unexpected shape for collision object");
        return false;
    }

    return success;
}

uint8_t SimpleCollisionObject::moveit_solid_primitive_type() const
{
    if (type == "box") {
        return shape_msgs::SolidPrimitive::BOX;
    }
    else if (type == "sphere") {
        return shape_msgs::SolidPrimitive::SPHERE;
    }
    else if (type == "cylinder") {
        return shape_msgs::SolidPrimitive::CYLINDER;
    }
    else if (type == "cone") {
        return shape_msgs::SolidPrimitive::CONE;
    }
    else {
        return (uint8_t)-1;
    }
}

std::vector<double> SimpleCollisionObject::moveit_solid_primitive_dims() const
{
    std::vector<double> moveit_dims(3);

    if (type == "box") {
        moveit_dims[shape_msgs::SolidPrimitive::BOX_X] = dims[BOX_LENGTH];
        moveit_dims[shape_msgs::SolidPrimitive::BOX_Y] = dims[BOX_WIDTH];
        moveit_dims[shape_msgs::SolidPrimitive::BOX_Z] = dims[BOX_HEIGHT];
        return moveit_dims;
    }
    else if (type == "sphere") {
        moveit_dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = dims[SPHERE_RADIUS];
        return moveit_dims;
    }
    else if (type == "cylinder") {
        moveit_dims[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dims[CYLINDER_HEIGHT];
        moveit_dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dims[CYLINDER_RADIUS];
        return moveit_dims;
    }
    else if (type == "cone") {
        moveit_dims[shape_msgs::SolidPrimitive::CONE_HEIGHT] = dims[CONE_HEIGHT];
        moveit_dims[shape_msgs::SolidPrimitive::CONE_RADIUS] = dims[CONE_RADIUS];
        return moveit_dims;
    }
    else {
        return moveit_dims;
    }
}

geometry_msgs::Pose SimpleCollisionObject::geometry_msgs_pose() const
{
    geometry_msgs::Pose p;
    p.position.x = pose.pos.x;
    p.position.y = pose.pos.y;
    p.position.z = pose.pos.z;

    Eigen::Affine3d rotmat(
            Eigen::AngleAxisd(pose.rot.y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose.rot.p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose.rot.r, Eigen::Vector3d::UnitX()));

    Eigen::Quaterniond quat(rotmat.rotation());

    p.orientation.w = quat.w();
    p.orientation.x = quat.x();
    p.orientation.y = quat.y();
    p.orientation.z = quat.z();

    return p;
}

StaticWorldSimulator::StaticWorldSimulator() :
    nh_(),
    ph_("~")
{
}

int StaticWorldSimulator::run()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    std::vector<SimpleCollisionObject> objects;
    if (!msg_utils::download_param(ph_, "collision_objects", objects)) {
        ROS_ERROR("Failed to retrieve param 'collision_objects'");
        return FAILED_TO_INITIALIZE;
    }

    ROS_INFO("Retrieved %zd collision objects", objects.size());

    // convert collision objects and publish add requests
    std::vector<moveit_msgs::CollisionObject> moveit_collision_objects;

    moveit_msgs::CollisionObject collision_object;
    for (const SimpleCollisionObject& object : objects) {
        collision_object.header.frame_id = "map";
        collision_object.header.stamp = ros::Time::now();
        collision_object.header.seq = 0;
        collision_object.id = object.id;
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = object.moveit_solid_primitive_type();
        collision_object.primitives[0].dimensions = object.moveit_solid_primitive_dims();
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0] = object.geometry_msgs_pose();
        collision_object.operation = moveit_msgs::CollisionObject::ADD;
    }

    return 0;
}
