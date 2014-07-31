#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <LinuxJoystick.h>
#include <ros/ros.h>
#include "teleop_node.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hdt_arm_teleop");
    ros::NodeHandle nh;

    hdt::TeleopNode node;
    return node.run();
}
