#include "WorldModelNode.h"

#include <moveit_msgs/PlanningSceneWorld.h>

WorldModelNode::WorldModelNode() :
    nh_(),
    ph_("~"),
    world_model_(),
    planning_scene_world_pub_()
{

}

int WorldModelNode::run()
{
    std::string world_frame;
    if (!ph_.getParam("world_frame", world_frame)) {
        ROS_ERROR("Failed to retrieve 'world_frame' from the param server");
        return FAILED_TO_INITIALIZE;
    }

    world_model_.reset(new WorldModel(world_frame));
    if (!world_model_) {
        ROS_ERROR("Failed to instantiate World Model");
        return FAILED_TO_INITIALIZE;
    }

    collision_objects_sub_ =
            nh_.subscribe("collision_objects", 10, &WorldModelNode::collision_objects_cb, this);

    octomap_sub_ = nh_.subscribe("octomap", 5, &WorldModelNode::octomap_cb, this);

    planning_scene_world_pub_ =
            nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene", 1, true);

    // Broadcast the current planning scene world
    ros::Rate loop_rate(1.0);
    while (ros::ok()) {
        ros::spinOnce();

        moveit_msgs::PlanningSceneWorld planning_scene_world;
        planning_scene_world_pub_.publish(planning_scene_world);

        loop_rate.sleep();
    }

    return SUCCESS;
}

void WorldModelNode::collision_objects_cb(const moveit_msgs::CollisionObject::ConstPtr& msg)
{

}

void WorldModelNode::octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{

}
