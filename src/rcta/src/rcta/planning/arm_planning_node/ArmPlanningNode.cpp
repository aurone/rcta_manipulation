#include "ArmPlanningNode.h"

namespace rcta {

ArmPlanningNode::ArmPlanningNode() :
    m_nh(),
    m_ph("~"),
    m_move_arm_command_server()
{
}

bool ArmPlanningNode::init()
{
    auto move_command_callback = boost::bind(&ArmPlanningNode::move_arm, this, _1);
    m_move_arm_command_server.reset(new MoveArmActionServer("move_arm_command", move_command_callback, false));
    if (!m_move_arm_command_server) {
        ROS_ERROR("Failed to instantiate Move Arm Action Server");
        return false;
    }

    m_move_arm_command_server->start();

    return true;
}

int ArmPlanningNode::run()
{
    ROS_INFO("Spinning...");
    ros::spin();
    ROS_INFO("Done spinning");
    return 0;
}

void ArmPlanningNode::move_arm(const rcta::MoveArmGoal::ConstPtr& request)
{
    geometry_msgs::PoseStamped wrist_goal_planning_frame;

    bool success = false;
    trajectory_msgs::JointTrajectory result_traj;
    if (request->type == rcta::MoveArmGoal::JointGoal) {
        ROS_INFO("Received a joint goal");
        moveit_msgs::RobotState goal_state;
        success = plan_to_joint_goal(goal_state, *request, result_traj);
    }
    else if (request->type == rcta::MoveArmGoal::EndEffectorGoal) {
        ROS_INFO("Received an end effector goal");
        success = plan_to_eef_goal(wrist_goal_planning_frame, result_traj);
    }

    if (!success) {
        rcta::MoveArmResult result;
        result.success = false;
        result.trajectory;
        m_move_arm_command_server->setAborted(result, "Failed to plan path");
        return;
    }

    if (request->execute_path) {
    }

    rcta::MoveArmResult result;
    result.success = true;
    result.trajectory = result_traj;
    m_move_arm_command_server->setSucceeded(result);
}

bool ArmPlanningNode::plan_to_eef_goal(
    const geometry_msgs::PoseStamped& goal_pose,
    trajectory_msgs::JointTrajectory& traj)
{
    return false;
}

bool ArmPlanningNode::plan_to_joint_goal(
    const moveit_msgs::RobotState& start,
    const rcta::MoveArmGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    return false;
}

} //namespace rcta
