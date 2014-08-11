#ifndef WorldSimulatorNode_h
#define WorldSimulatorNode_h

#include <cstdint>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

struct EulerAngles
{
    double r, p, y;
};

struct SixDofPose
{
    geometry_msgs::Point pos;
    EulerAngles rot;
};

struct SimpleCollisionObject
{
    std::string id;
    std::string type;
    SixDofPose pose;

    static const std::size_t BOX_LENGTH = 0, BOX_WIDTH = 1, BOX_HEIGHT = 2;
    static const std::size_t SPHERE_RADIUS = 0;
    static const std::size_t CYLINDER_HEIGHT = 0, CYLINDER_RADIUS = 1;
    static const std::size_t CONE_HEIGHT = 0, CONE_RADIUS = 1;

    // shape-specific dimensions
    double dims[3];

    uint8_t moveit_solid_primitive_type() const;
    std::vector<double> moveit_solid_primitive_dims() const;
    geometry_msgs::Pose geometry_msgs_pose() const;
};

class StaticWorldSimulator
{
public:

    StaticWorldSimulator();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher collision_object_pub_;
};

#endif
