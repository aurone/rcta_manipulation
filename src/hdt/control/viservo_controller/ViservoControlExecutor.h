#ifndef ViservoControlExecutor_h
#define ViservoControlExecutor_h

#include <memory>
#include <mutex>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <sensor_msgs/JointState.h>
#include <hdt/ViservoCommandAction.h>

class ViservoControlExecutor
{
public:

    ViservoControlExecutor();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    typedef actionlib::SimpleActionServer<hdt::ViservoCommandAction> ViservoCommandActionServer;
    std::unique_ptr<ViservoCommandActionServer> as_;

    std::string action_name_;

    ros::Publisher joint_command_pub_;
    ros::Subscriber joint_states_sub_;

    std::mutex msg_mutex;

    hdt::ViservoCommandGoal::ConstPtr current_goal_;

    sensor_msgs::JointState::ConstPtr last_joint_state_msg_;
    ar_track_alvar::AlvarMarkers::ConstPtr last_ar_markers_msg_;

    void goal_callback();
    void preempt_callback();
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void ar_markers_cb(const ar_track_alvar::AlvarMarkers& msg);
};

#endif
