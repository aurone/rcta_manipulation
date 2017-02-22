#ifndef GripperAction_h
#define GripperAction_h

// standard includes
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// system includes
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>

// project includes
#include <rcta/control/robotiq_controllers/gripper_model.h>

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

    control_msgs::GripperCommandGoal::ConstPtr curr_goal_;

    ros::Time last_gripper_connect_attempt_time_;

    GripperModel gripper_model_;

    void execute_callback(const control_msgs::GripperCommandGoalConstPtr& goal);

    void goal_callback();
    void preempt_callback();

    unsigned long resolve_to_ipv4(const std::string& hostname, uint16_t portno);
};

} // robotiq

#endif
