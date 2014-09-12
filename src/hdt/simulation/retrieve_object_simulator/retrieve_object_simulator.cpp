#include <ros/ros.h>
#include "RetrieveObjectSimulator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieve_object_simulator");
    return RetrieveObjectSimulator().run();
}
