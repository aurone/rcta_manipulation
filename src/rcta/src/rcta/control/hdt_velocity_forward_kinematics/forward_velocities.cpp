// standard includes
#include <cstdio>
#include <string>

// system includes
#include <boost/shared_ptr.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>

// project includes
#include <rcta/common/hdt_description/RobotModel.h>

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<const urdf::Link> LinkConstPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<const urdf::Joint> JointConstPtr;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "forward_velocities");
    ros::NodeHandle nh;
    std::string urdf_string;
    if (!nh.getParam("robot_description", urdf_string)) {
        ROS_WARN("Failed to retrieve 'robot_description' from the param server");
        return 1;
    }

    boost::shared_ptr<urdf::ModelInterface> model = urdf::parseURDF(urdf_string);
    if (!model) {
        ROS_WARN("Failed to parse URDF from XML string");
        return 1;
    }

    const std::string arm_link_1_name = "arm_1_shoulder_twist_link";
    const std::string gripper_base_link_name = "gripper_base";

    LinkConstPtr first_arm_link = model->getLink(arm_link_1_name);
    LinkConstPtr mount_link = first_arm_link->getParent();
    LinkConstPtr gripper_link = model->getLink(gripper_base_link_name);

    if (!first_arm_link || !mount_link || !gripper_link) {
        ROS_ERROR("Some required link doesn't exist");
        return 1;
    }

    ROS_INFO("Found first_arm_link @ %p", first_arm_link.get());
    ROS_INFO("Found mount_link @ %p", mount_link.get());
    ROS_INFO("Found gripper_link @ %p", gripper_link.get());

    // Build the kinematic chain
    KDL::Chain hdt_chain;

    LinkConstPtr curr_link = mount_link;
    while (curr_link != gripper_link) {
        ROS_INFO("Current Link: %s", curr_link->name.c_str());
        JointConstPtr joint = curr_link->child_joints.front();
        double ax = joint->parent_to_joint_origin_transform.position.x;
        double ay = joint->parent_to_joint_origin_transform.position.y;
        double az = joint->parent_to_joint_origin_transform.position.z;
        hdt_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(ax, ay, az))));
        curr_link = model->getLink(curr_link->child_joints.front()->child_link_name);
    }

    KDL::ChainFkSolverVel_recursive fv_solver(hdt_chain);
    KDL::JntArray q(hdt_chain.getNrOfJoints());
    q(0, 0) = 0.0;
    q(1, 0) = 0.0;
    q(2, 0) = 0.0;
    q(3, 0) = 0.0;
    q(4, 0) = 0.0;
    q(5, 0) = 0.0;
    q(6, 0) = 0.0;

    KDL::JntArrayVel velocities(q);

    KDL::FrameVel out;
    fv_solver.JntToCart(velocities, out);

    double px = out.p.p(0);
    double py = out.p.p(1);
    double pz = out.p.p(2);
    double vx = out.p.v(0);
    double vy = out.p.v(1);
    double vz = out.p.v(2);

    ROS_INFO("(%0.3f, %0.3f, %0.3f) @ (%0.3f, %0.3f, %0.3f)", px, py, pz, vx, vy, vz);

    return 0;
}
