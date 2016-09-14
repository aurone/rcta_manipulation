#include <ros/ros.h>
#include "ArmPlanningNode.h"

enum MainResult
{
    SUCCESSFUL_TERMINATION = 0,
    FAILED_TO_INITIALIZE_NODE,
    UNSUCCESSFUL_TERMINATION
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hdt_arm_planning_node");

    rcta::ArmPlanningNode node;

    if (!node.init()) {
        return FAILED_TO_INITIALIZE_NODE;
    }

    if (0 != node.run()) {
        return UNSUCCESSFUL_TERMINATION;
    }
    else {
        return SUCCESSFUL_TERMINATION;
    }
}
