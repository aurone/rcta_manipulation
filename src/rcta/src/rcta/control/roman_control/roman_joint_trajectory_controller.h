#ifndef ROMAN_JOINT_TRAJECTORY_CONTROLLER_H
#define ROMAN_JOINT_TRAJECTORY_CONTROLLER_H

#include <map>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <roman_client_ros_utils/RomanSpecReply.h>
#include <roman_client_ros_utils/RomanState.h>
#include <ros/ros.h>

#define BUILD_LIVE_JPL 1

class RomanJointTrajectoryController
{
public:

    RomanJointTrajectoryController(const std::string& ns);

    ~RomanJointTrajectoryController();

    int run();

private:

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
    FollowJointTrajectoryActionServer;

    ros::NodeHandle m_nh;

    FollowJointTrajectoryActionServer m_server;

    ros::Publisher m_roman_spec_pub;
    ros::Subscriber m_roman_spec_reply_sub;
    ros::Subscriber m_state_sub;

    FollowJointTrajectoryActionServer::GoalConstPtr m_goal;
    FollowJointTrajectoryActionServer::Feedback m_feedback;
    FollowJointTrajectoryActionServer::Result m_result;

    typedef roman_client_ros_utils::RomanState RomanState;
    typedef roman_client_ros_utils::RomanSpecReply RomanSpecReply;

    RomanState::ConstPtr m_state;

    std::map<std::string, int> m_joint_name_to_spec_index;

    void romanStateCallback(const RomanState::ConstPtr& msg);

    void goalCallback();

    void preemptCallback();

    void romanSpecReplyCallback(const RomanSpecReply::ConstPtr& msg);
};

#endif
