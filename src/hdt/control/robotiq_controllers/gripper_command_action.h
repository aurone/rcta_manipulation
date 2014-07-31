#ifndef GripperAction_h
#define GripperAction_h

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>

class GripperInterface;
class GripperConnection;

namespace robotiq
{

class GripperCommandActionExecutor
{
    typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperCommandActionServer;
    typedef GripperCommandActionServer::GoalHandle GoalHandle;

public:

    GripperCommandActionExecutor(ros::NodeHandle& nh);
    ~GripperCommandActionExecutor();

    enum RunResult
    {
        SUCCESS,
        FAILED_TO_INITIALIZE
    };
    static std::string to_string(RunResult result);
    RunResult run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::string action_server_name_;
    std::unique_ptr<GripperCommandActionServer> action_server_;

    control_msgs::GripperCommandFeedback feedback_;
    control_msgs::GripperCommandResult result_;

    std::shared_ptr<GripperConnection> connection_;
    std::shared_ptr<GripperInterface> gripper_;

    double gripper_throttle_rate_hz_;

    ros::Publisher joint_state_pub_;

    std::mutex gripper_mutex_;

    void goal_callback(const control_msgs::GripperCommandGoalConstPtr& goal);

    unsigned long resolve_to_ipv4(const std::string& hostname, uint16_t portno);
};

} // robotiq

#endif
