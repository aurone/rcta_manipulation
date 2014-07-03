#include <ros/ros.h>
#include "JointTrajectoryExecutor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action");
    ros::NodeHandle node;
    hdt::JointTrajectoryExecutor jte(node);
    if (!jte.initialize()) {
        ROS_ERROR("Failed to initialize Joint Trajectory Executor");
        ros::shutdown();
        return 1;
    }

    ros::spin();

    return 0;
}

