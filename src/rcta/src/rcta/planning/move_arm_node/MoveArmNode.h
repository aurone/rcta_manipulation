#ifndef rcta_move_arm_node_h
#define rcta_move_arm_node_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <rcta/MoveArmAction.h>

namespace rcta {

/// @brief Implements a ROS node to provide planning and execution of paths
///
/// The ROS node maintains the state of the robot and the world to plan and
/// and execute paths that avoid obstacles in the environment and self
/// collisions with the robot itself.
class MoveArmNode
{
public:

    MoveArmNode();

    bool init();

    int run();

private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    typedef actionlib::SimpleActionServer<rcta::MoveArmAction> MoveArmActionServer;
    std::string m_server_name;
    std::unique_ptr<MoveArmActionServer> m_move_arm_server;

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupActionClient;
    std::unique_ptr<MoveGroupActionClient> m_move_group_client;

    std::string m_tip_link;
    double m_pos_tolerance;
    double m_rot_tolerance;
    double m_joint_tolerance;

    moveit_msgs::MoveGroupGoal m_goal;
    moveit_msgs::MoveGroupResult m_result;

    void moveArm(const rcta::MoveArmGoal::ConstPtr& goal);

    bool sendMoveGroupPoseGoal(
        const moveit_msgs::PlanningOptions& ops,
        const geometry_msgs::PoseStamped& goal_pose);

    bool planToGoalEE(
        const geometry_msgs::PoseStamped& goal_pose,
        trajectory_msgs::JointTrajectory& traj);

    bool planToGoalJoints(
        const moveit_msgs::RobotState& start,
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    bool moveToGoalEE(
        const geometry_msgs::PoseStamped& goal_pose,
        trajectory_msgs::JointTrajectory& traj);

    bool moveToGoalJoints(
        const moveit_msgs::RobotState& start,
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    void moveGroupResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result);
};

} // namespace rcta

#endif
