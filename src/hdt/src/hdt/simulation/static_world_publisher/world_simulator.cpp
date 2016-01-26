#include "WorldSimulatorNode.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "static_world_publisher");
    return StaticWorldSimulator().run();
}
