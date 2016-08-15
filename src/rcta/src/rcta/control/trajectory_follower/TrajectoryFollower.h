#ifndef TrajectoryFollower_h
#define TrajectoryFollower_h

// standard includes
#include <string>

// system includes
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <rcta/common/hdt_description/RobotModel.h>

class TrajectoryFollower
{
public:

    TrajectoryFollower();
    ~TrajectoryFollower();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    MainResult run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    hdt::RobotModelPtr robot_model_;

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;
    std::unique_ptr<FollowJointTrajectoryActionServer> as_;
    std::string action_server_name_;

    control_msgs::FollowJointTrajectoryGoal current_goal_;
    ros::Time execution_start_time_;

    ros::Publisher joint_traj_pub_;

    bool initialize();
    void goal_callback();
    void preempt_callback();

    bool check_goal(const control_msgs::FollowJointTrajectoryGoal& goal) const;

    control_msgs::FollowJointTrajectoryGoal retime_trajectory(const control_msgs::FollowJointTrajectoryGoal& goal);

    bool get_point_at_time(
        const control_msgs::FollowJointTrajectoryGoal& joint_traj,
        const ros::Time& time_from_start,
        trajectory_msgs::JointTrajectoryPoint& traj_pt) const;
};

#endif
