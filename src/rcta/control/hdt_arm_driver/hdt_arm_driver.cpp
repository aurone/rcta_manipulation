#include <memory>
#include <ros/ros.h>
#include "manipulator_interface_live_ros.h"
#include "manipulator_interface_sim_ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hdt_arm_driver");

    ros::NodeHandle ph("~");
    bool sim;
    ph.param("sim", sim, false);

    const bool live = !sim;

    using namespace hdt;

    std::unique_ptr<ManipulatorInterfaceROS> manip_interface;

    if (live) {
        ROS_INFO("Instantiating Manipulator Interface Live ROS");
        manip_interface.reset(new ManipulatorInterfaceLiveROS);
    }
    else {
        ROS_INFO("Instantiating Manipulator Interface Sim ROS");
        manip_interface.reset(new ManipulatorInterfaceSimROS);
    }

    if (!manip_interface) {
        ROS_ERROR("Failed to instantiate Manipulator Interface ROS");
        return 2;
    }

    ManipulatorInterfaceROS::RunResult res = manip_interface->run();
    if (res != ManipulatorInterfaceROS::SUCCESS) {
        ROS_ERROR("HDT Manipulator ROS Interface exited with error %s", ManipulatorInterfaceROS::RunResultToString(res));
        if (res == ManipulatorInterfaceROS::FAILED_TO_INITIALIZE) {
            ROS_ERROR("Initialization failure (%s)", to_string(manip_interface->init_error()).c_str());
        }
        return 1;
    }

    return 0;
}
