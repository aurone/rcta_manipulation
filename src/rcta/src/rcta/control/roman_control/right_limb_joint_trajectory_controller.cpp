#include "roman_joint_trajectory_controller.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_joint_trajectory_controller");
    return RomanJointTrajectoryController("right_limb").run();
}
