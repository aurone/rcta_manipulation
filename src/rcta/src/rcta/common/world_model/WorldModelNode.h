#ifndef WorldModelNode_h
#define WorldModelNode_h

#include <memory>
#include <actionlib/server/action_server.h>
#include <moveit_msgs/CollisionObject.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include "WorldModel.h"

/// @brief Provides a ROS API to manipulating a common world model.
///        Updates to the world model occur through an action server interface.
///        Broadcasting of updates occurs through a latched ROS topic interface.
class WorldModelNode
{
public:

    WorldModelNode();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::unique_ptr<WorldModel> world_model_;

    ros::Subscriber collision_objects_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher planning_scene_world_pub_;

    void collision_objects_cb(const moveit_msgs::CollisionObject::ConstPtr& msg);
    void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg);
};

#endif
