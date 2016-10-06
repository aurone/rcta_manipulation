#ifndef ROMAN_GRIPPER_CONTROLLER_H
#define ROMAN_GRIPPER_CONTROLLER_H

// system includes
#include <actionlib/server/simple_action_server.h>
#include <boost/circular_buffer.hpp>
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

    typedef roman_client_ros_utils::RomanState RomanState;

    double m_expected_state_rate;   ///< Expected publish rate of RomanState
    double m_state_history_length;  ///< Length, in seconds, of states to keep
    boost::circular_buffer<RomanState::ConstPtr> m_state_hist;

    enum Hand
    {
        Right = 0,
        Left
    } m_hand;

    void goalCallback();
    void preemptCallback();

    void romanStateCallback(const RomanState::ConstPtr& msg);
};

#endif
