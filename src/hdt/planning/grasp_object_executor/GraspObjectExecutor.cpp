#include "GraspObjectExecutor.h"

#include <hdt/common/utils/RunUponDestruction.h>

GraspObjectExecutor::GraspObjectExecutor() :
    nh_(),
    ph_("~")
{

}

int GraspObjectExecutor::run()
{
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        RunUponDestruction rod([&](){ loop_rate.sleep(); });

        // receive pose of the object
        // idle ->
        // generating grasps ->
        // arm planning and execution to pregrasp ->
        // visual servoing to the pregrasp ->
        // visual servoing to the grasp ->
        // arm planning and execution to egress position ->
        // idle
    }
    return SUCCESS;
}

void GraspObjectExecutor::goal_callback()
{

}

void GraspObjectExecutor::preempt_callback()
{

}
