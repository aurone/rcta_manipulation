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
#include <octomap_msgs/Octomap.h>
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

    std::string m_model_frame;

    ros::Subscriber m_octomap_sub;

    typedef actionlib::SimpleActionServer<rcta::MoveArmAction> MoveArmActionServer;
    std::string m_server_name;
    std::unique_ptr<MoveArmActionServer> m_move_arm_server;

    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupActionClient;
    std::unique_ptr<MoveGroupActionClient> m_move_group_client;

    std::string m_tip_link;
    double m_pos_tolerance;
    double m_rot_tolerance;
    double m_joint_tolerance;

    // Goal shared between plan/execute requests so that parameters inherited
    // from config don't have to be set every time.
    moveit_msgs::MoveGroupGoal m_goal;

    moveit_msgs::MoveGroupResult m_result;

    ros::AsyncSpinner m_spinner;

    octomap_msgs::Octomap::ConstPtr m_octomap;

    void moveArm(const rcta::MoveArmGoal::ConstPtr& goal);

    bool planToGoalEE(
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    bool planToGoalJoints(
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    bool moveToGoalEE(
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    bool moveToGoalJoints(
        const rcta::MoveArmGoal& goal,
        trajectory_msgs::JointTrajectory& traj);

    // setup planning options for the current request, including slerping over
    // the most recent octomap
    void fillPlanOnlyOptions(
        const rcta::MoveArmGoal& goal,
        moveit_msgs::PlanningOptions& ops) const;

    void fillPlanAndExecuteOptions(
        const rcta::MoveArmGoal& goal,
        moveit_msgs::PlanningOptions& ops) const;

    void fillCommonOptions(
        const rcta::MoveArmGoal& goal,
        moveit_msgs::PlanningOptions& ops) const;

    bool sendMoveGroupPoseGoal(
        const moveit_msgs::PlanningOptions& ops,
        const rcta::MoveArmGoal& goal);

    bool sendMoveGroupConfigGoal(
        const moveit_msgs::PlanningOptions& ops,
        const rcta::MoveArmGoal& goal);

    void moveGroupResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result);

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    moveit_msgs::CollisionObject createGroundPlaneObject() const;
};

} // namespace rcta

#endif
