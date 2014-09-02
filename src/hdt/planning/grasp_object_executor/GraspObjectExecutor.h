#ifndef ObjectPickupExecutor_h
#define ObjectPickupExecutor_h

#include <actionlib/server/simple_action_server.h>

// Handles requests for PickupObject actions
class GraspObjectExecutor
{
public:

    GraspObjectExecutor();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    void goal_callback();
    void preempt_callback();
};

#endif
