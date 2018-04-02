#ifndef ROMAN_JOINT_TRAJECTORY_CONTROLLER_H
#define ROMAN_JOINT_TRAJECTORY_CONTROLLER_H

// standard includes
#include <map>
#include <string>

// system includes
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <roman_client_ros_utils/RomanSpecReply.h>
#include <roman_client_ros_utils/RomanState.h>
#include <ros/ros.h>

class RomanJointTrajectoryController
{
public:

    using FollowJointTrajectoryActionServer =
            actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;

    using FollowJointTrajectoryActionClient =
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ah;

    FollowJointTrajectoryActionServer m_server;
    FollowJointTrajectoryActionClient m_client;

    ros::Publisher m_roman_spec_pub;
    ros::Subscriber m_roman_spec_reply_sub;
    ros::Subscriber m_state_sub;

    FollowJointTrajectoryActionServer::GoalConstPtr m_goal;
    FollowJointTrajectoryActionServer::Feedback m_feedback;
    FollowJointTrajectoryActionServer::Result m_result;

    typedef roman_client_ros_utils::RomanState RomanState;
    typedef roman_client_ros_utils::RomanSpecReply RomanSpecReply;

    RomanState::ConstPtr m_state;

    std::vector<std::string> spec_index_to_joint_name;
    std::map<std::string, int> m_joint_name_to_spec_index;

    void romanStateCallback(const RomanState::ConstPtr& msg);

    void goalCallback();

    void preemptCallback();

    void romanSpecReplyCallback(const RomanSpecReply::ConstPtr& msg);

    RomanJointTrajectoryController(const std::string& ns);
    ~RomanJointTrajectoryController();
    int run();
};

#endif
