#ifndef ROMAN_GRIPPER_CONTROLLER_H
#define ROMAN_GRIPPER_CONTROLLER_H

// system includes
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <roman_client_ros_utils/RomanState.h>
#include <ros/ros.h>

class RomanGripperController
{
public:

    RomanGripperController();
    ~RomanGripperController();

    int run();

private:

    typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction>
    GripperCommandActionServer;

    ros::NodeHandle m_nh;

    GripperCommandActionServer m_server;

    ros::Publisher m_hand_activate_pub;
    ros::Publisher m_hand_goto_pub;
    ros::Publisher m_hand_open_pub;
    ros::Publisher m_hand_close_pub;

    ros::Subscriber m_roman_state_sub;

    GripperCommandActionServer::GoalConstPtr m_goal;
    GripperCommandActionServer::Feedback m_feedback;
    GripperCommandActionServer::Result m_result;

    roman_client_ros_utils::RomanState::ConstPtr m_state;

    void goalCallback();
    void preemptCallback();

    void romanStateCallback(
        const roman_client_ros_utils::RomanState::ConstPtr& msg);
};

#endif
