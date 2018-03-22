#include <ros/ros.h>
#include "PickAndPlaceNode.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hdt_pick_and_place");

    PickAndPlaceNode node;
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize Pick And Place Node");
        return 1;
    }
    return node.run();
}
