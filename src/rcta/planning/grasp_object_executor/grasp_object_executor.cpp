#include <ros/ros.h>
#include "GraspObjectExecutor.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_pickup_executor");
    return GraspObjectExecutor().run();
}
