#include "TrajectoryFollower.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "trajectory_follower");
    return TrajectoryFollower().run();
}
