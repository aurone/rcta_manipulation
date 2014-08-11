#ifndef DepthSensorSimulatorNode_h
#define DepthSensorSimulatorNode_h

#include <moveit_msgs/PlanningSceneWorld.h>
#include <ros/ros.h>

class DepthSensorSimulatorNode
{
public:

    DepthSensorSimulatorNode();
    ~DepthSensorSimulatorNode();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run(int argc, char* argv[]);

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Subscriber planning_scene_world_sub_;
    ros::Publisher point_cloud_pub_;

    moveit_msgs::PlanningSceneWorld::ConstPtr last_planning_scene_world_;

    void planning_scene_world_cb(const moveit_msgs::PlanningSceneWorld::ConstPtr& msg);
};

#endif
