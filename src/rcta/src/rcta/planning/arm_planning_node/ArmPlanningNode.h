#ifndef hdt_ArmPlanningNode_h
#define hdt_ArmPlanningNode_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <rcta/MoveArmCommandAction.h>

namespace rcta {

/// @brief Implements a ROS node to provide planning and execution of paths
///
/// The ROS node maintains the state of the robot and the world to plan and
/// and execute paths that avoid obstacles in the environment and self
/// collisions with the robot itself.
class ArmPlanningNode
{
public:

    ArmPlanningNode();

    bool init();

    int run();

private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    typedef actionlib::SimpleActionServer<rcta::MoveArmCommandAction> MoveArmActionServer;
    std::unique_ptr<MoveArmActionServer> m_move_arm_command_server;

    void move_arm(const rcta::MoveArmCommandGoal::ConstPtr& goal);

    bool plan_to_eef_goal(
            const geometry_msgs::PoseStamped& goal_pose,
            trajectory_msgs::JointTrajectory& traj);

    bool plan_to_joint_goal(
            const moveit_msgs::RobotState& start,
            const rcta::MoveArmCommandGoal& goal,
            trajectory_msgs::JointTrajectory& traj);
};

} // namespace rcta

#endif
