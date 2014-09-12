#include <ros/ros.h>
#include "LocalizationSimulator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localization_simulator");
    return LocalizationSimulator().run();
}
