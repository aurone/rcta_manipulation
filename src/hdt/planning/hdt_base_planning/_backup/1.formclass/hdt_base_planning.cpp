#include <ros/ros.h>
#include "RepositionBaseExecutor.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hdt_base_planning");
    return RepositionBaseExecutor().run();
}
