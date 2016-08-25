// system includes
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <roman_client_ros_utils/RomanSpec.h>
#include <roman_client_ros_utils/RomanSpecReply.h>
#include <ros/ros.h>

// project includes
#include "roman_joint_trajectory_controller.h"

class RomanJointTrajectoryController
{
public:

    RomanJointTrajectoryController() :
        m_nh(),
        m_server(
            ros::NodeHandle("right_limb"), "follow_joint_trajectory", false),
        m_roman_spec_pub(),
        m_roman_spec_reply_sub()
    {
        m_roman_spec_pub = m_nh.advertise<roman_client_ros_utils::RomanSpec>(
                "roman_spec", 1);
        m_roman_spec_reply_sub = m_nh.subscribe(
                "roman_spec_reply", 1,
                &RomanJointTrajectoryController::romanSpecReplyCallback, this);

        m_server.registerGoalCallback(boost::bind(&RomanJointTrajectoryController::goalCallback, this));
        m_server.registerPreemptCallback(boost::bind(&RomanJointTrajectoryController::preemptCallback, this));
    }

    ~RomanJointTrajectoryController()
    {
        if (m_server.isActive()) {
            m_server.setAborted();
        }
        m_server.shutdown();
    }

    int run()
    {
        m_server.start();
        ros::spin();
        return 0;
    }

private:

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
    FollowJointTrajectoryActionServer;

    ros::NodeHandle m_nh;

    FollowJointTrajectoryActionServer m_server;

    ros::Publisher m_roman_spec_pub;
    ros::Subscriber m_roman_spec_reply_sub;

    FollowJointTrajectoryActionServer::GoalConstPtr m_goal;
    FollowJointTrajectoryActionServer::Feedback m_feedback;
    FollowJointTrajectoryActionServer::Result m_result;

    void goalCallback()
    {
        ROS_INFO("This is probably where i would publish the spec");
        m_goal = m_server.acceptNewGoal();
        // TODO: publish the path message
    }

    void preemptCallback()
    {
        ROS_INFO("Preempt action goal");
        // TODO: send halt message to the arm
        m_server.setPreempted();
    }

    void romanSpecReplyCallback(
        const roman_client_ros_utils::RomanSpecReply::ConstPtr& msg)
    {
        if (m_server.isActive()) {
            if (!msg->attempted || msg->in_fault) {
                m_result.error_code = FollowJointTrajectoryActionServer::Result::INVALID_GOAL;
                m_server.setAborted(m_result);
            }
            else {
                m_result.error_code = FollowJointTrajectoryActionServer::Result::SUCCESSFUL;
                m_server.setSucceeded(m_result);
            }
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_joint_trajectory_controller");
    return RomanJointTrajectoryController().run();
}
