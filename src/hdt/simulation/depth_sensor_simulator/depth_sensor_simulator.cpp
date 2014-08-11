#include <SDL.h>
#include <ros/ros.h>
#include "DepthSensorSimulatorNode.h"

int main(int argc, char* argv[])
{
    SDL_SetMainReady();
    ros::init(argc, argv, "depth_sensor_simulator");
    return DepthSensorSimulatorNode().run(argc, argv);
}
