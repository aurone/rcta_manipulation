#include "MoveArmNode.h"

#include <sbpl_geometry_utils/utils.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>

namespace rcta {

MoveArmNode::MoveArmNode() :
    m_nh(),
    m_ph("~"),
    m_server_name("move_arm"),
    m_move_arm_server(),
    m_spinner(2)
{
}

bool MoveArmNode::init()
{
    auto move_arm_callback = boost::bind(&MoveArmNode::moveArm, this, _1);
    m_move_arm_server.reset(
            new MoveArmActionServer(m_server_name, move_arm_callback, false));

    m_move_group_client.reset(new MoveGroupActionClient("move_group", false));

    ///////////////////////////////////////
    // Load Common Move Group Parameters //
    ///////////////////////////////////////

    // planner settings
    double allowed_planning_time;
    std::string planner_id;

    // goal settings
    std::string group_name;

    double pos_tolerance;
    double rot_tolerance_deg;
    double joint_tolerance_deg;
    std::string tip_link;

    std::string workspace_frame;
    geometry_msgs::Vector3 workspace_min;
    geometry_msgs::Vector3 workspace_max;

    m_ph.param("allowed_planning_time", allowed_planning_time, 10.0);
    if (!m_ph.getParam("planner_id", planner_id)) {
        ROS_ERROR("Failed to retrieve 'planner_id' from the param server");
        return false;
    }

    if (!m_ph.getParam("group_name", group_name)) {
        ROS_ERROR("Failed to retrieve 'group_name' from the param server");
        return false;
    }
    m_ph.param("pos_tolerance", pos_tolerance, 0.05);
    m_ph.param("rot_tolerance", rot_tolerance_deg, 5.0);
    m_ph.param("joint_tolerance", joint_tolerance_deg, 5.0);
    if (!m_ph.getParam("tip_link", tip_link)) {
        ROS_ERROR("Failed to retrieve 'tip_link' from the param server");
        return false;
    }

    if (!m_ph.getParam("workspace_frame", workspace_frame)) {
        ROS_ERROR("Failed to retrieve 'workspace_frame' from the param server");
        return false;
    }
    if (!msg_utils::download_param(m_ph, "workspace_min", workspace_min) ||
        !msg_utils::download_param(m_ph, "workspace_max", workspace_max))
    {
        return false;
    }

    m_goal.request.allowed_planning_time = allowed_planning_time;
    m_goal.request.planner_id = planner_id;

    m_goal.request.group_name = group_name;

    m_goal.request.max_acceleration_scaling_factor = 1.0;
    m_goal.request.max_velocity_scaling_factor = 1.0;
    m_goal.request.num_planning_attempts = 1;
    m_goal.request.path_constraints.joint_constraints.clear();
    m_goal.request.path_constraints.name = "";
    m_goal.request.path_constraints.orientation_constraints.clear();
    m_goal.request.path_constraints.position_constraints.clear();
    m_goal.request.path_constraints.visibility_constraints.clear();
    m_goal.request.trajectory_constraints.constraints.clear();
    m_goal.request.start_state.is_diff = true;

    m_tip_link = tip_link;
    m_pos_tolerance = pos_tolerance;
    m_rot_tolerance = sbpl::utils::ToRadians(rot_tolerance_deg);
    m_joint_tolerance = sbpl::utils::ToRadians(joint_tolerance_deg);

    m_goal.request.workspace_parameters.header.frame_id = workspace_frame;
    m_goal.request.workspace_parameters.min_corner = workspace_min;
    m_goal.request.workspace_parameters.max_corner = workspace_max;

    ROS_INFO("Allowed Planning Time: %0.3f", allowed_planning_time);
    ROS_INFO("Planner ID: %s", planner_id.c_str());
    ROS_INFO("Group Name: %s", group_name.c_str());
    ROS_INFO("Position Tolerance (m): %0.3f", pos_tolerance);
    ROS_INFO("Rotation Tolerance (deg): %0.3f", rot_tolerance_deg);
    ROS_INFO("Joint Tolerance (deg): %0.3f", joint_tolerance_deg);
    ROS_INFO("Tip Link: %s", tip_link.c_str());
    ROS_INFO("Workspace Frame: %s", workspace_frame.c_str());
    ROS_INFO("Workspace Min: (%0.3f, %0.3f, %0.3f)", workspace_min.x, workspace_min.y, workspace_min.z);
    ROS_INFO("Workspace Max: (%0.3f, %0.3f, %0.3f)", workspace_max.x, workspace_max.y, workspace_max.z);

    return true;
}

int MoveArmNode::run()
{
    ROS_INFO("Spinning...");
    m_move_arm_server->start();
    m_spinner.start();
    ros::waitForShutdown();
    ROS_INFO("Done spinning");
    return 0;
}

void MoveArmNode::moveArm(const rcta::MoveArmGoal::ConstPtr& request)
{
    geometry_msgs::PoseStamped tip_goal;

    // TODO: get this from the configured planning frame in moveit
    tip_goal.header.frame_id = "map";

    tip_goal.pose = request->goal_pose;

    bool success = false;
    trajectory_msgs::JointTrajectory result_traj;
    const bool execute = request->execute_path;

    if (request->type == rcta::MoveArmGoal::JointGoal) {
        if (execute) {
            moveit_msgs::RobotState goal_state;
            success = moveToGoalJoints(goal_state, *request, result_traj);
        } else {
            moveit_msgs::RobotState goal_state;
            success = planToGoalJoints(goal_state, *request, result_traj);
        }
    } else if (request->type == rcta::MoveArmGoal::EndEffectorGoal) {
        if (execute) {
            success = moveToGoalEE(tip_goal, result_traj);
        } else {
            success = planToGoalEE(tip_goal, result_traj);
        }
    } else {
        ROS_ERROR("Unrecognized goal type");
        rcta::MoveArmResult result;
        result.success = false;
        m_move_arm_server->setAborted(result, "Unrecognized goal type");
    }

    if (success) {
        rcta::MoveArmResult result;
        result.success = true;
        result.trajectory = result_traj;
        m_move_arm_server->setSucceeded(result);
    } else {
        rcta::MoveArmResult result;
        result.success = false;
        result.trajectory;
        m_move_arm_server->setAborted(result, "Failed to plan path");
        return;
    }
}

/// \brief Send a move group request with goal pose constraints
///
/// The actual result of the request (whether the planning/execution completed
/// successfully) should be checked by examining the value of \p m_result.
///
/// \return true if the goal was sent to the server; false otherwise
bool MoveArmNode::sendMoveGroupPoseGoal(
    const moveit_msgs::PlanningOptions& ops,
    const geometry_msgs::PoseStamped& goal_pose)
{
    if (!m_move_group_client->isServerConnected()) {
        ROS_ERROR("server is not connected");
        return false;
    }

    moveit_msgs::MotionPlanRequest& req = m_goal.request;

    req.goal_constraints.clear();

    // one set of goal constraints
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    // one position constraint
    moveit_msgs::PositionConstraint goal_pos_constraint;
    goal_pos_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_pos_constraint.link_name = m_tip_link;
    goal_pos_constraint.target_point_offset = geometry_msgs::CreateVector3(0.0, 0.0, 0.0);
    shape_msgs::SolidPrimitive tolerance_volume;
    tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    tolerance_volume.dimensions = { m_pos_tolerance };
    goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
    goal_pos_constraint.constraint_region.primitive_poses.push_back(goal_pose.pose);
    goal_pos_constraint.weight = 1.0;

    // one orientation constraint
    moveit_msgs::OrientationConstraint goal_rot_constraint;
    goal_rot_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_rot_constraint.orientation = goal_pose.pose.orientation;
    goal_rot_constraint.link_name = m_tip_link;
    goal_rot_constraint.absolute_x_axis_tolerance = m_rot_tolerance;
    goal_rot_constraint.absolute_y_axis_tolerance = m_rot_tolerance;
    goal_rot_constraint.absolute_z_axis_tolerance = m_rot_tolerance;
    goal_rot_constraint.weight = 1.0;

    goal_constraints.position_constraints.push_back(goal_pos_constraint);
    goal_constraints.orientation_constraints.push_back(goal_rot_constraint);

    req.goal_constraints.push_back(goal_constraints);

    m_goal.planning_options = ops;

    auto result_callback = boost::bind(
            &MoveArmNode::moveGroupResultCallback, this, _1, _2);
    m_move_group_client->sendGoal(m_goal, result_callback);

    if (!m_move_group_client->waitForResult()) {
        return false;
    }

    return true;
}

void MoveArmNode::moveGroupResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const moveit_msgs::MoveGroupResult::ConstPtr& result)
{
    if (result) {
        ROS_INFO("receive result from move_group");
        m_result = *result;
    }
}

bool MoveArmNode::planToGoalEE(
    const geometry_msgs::PoseStamped& goal_pose,
    trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO("plan to goal pose");
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = true;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    if (!sendMoveGroupPoseGoal(ops, goal_pose)) {
        return false;
    }

    if (m_result.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        return false;
    } else {
        // TODO: slerp trajectory
        return true;
    }
}

bool MoveArmNode::planToGoalJoints(
    const moveit_msgs::RobotState& start,
    const rcta::MoveArmGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO("plan to goal joints");
    return false;
}

bool MoveArmNode::moveToGoalEE(
    const geometry_msgs::PoseStamped& goal_pose,
    trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO("move to goal pose");
    moveit_msgs::PlanningOptions ops;
    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.look_around = false;
    ops.look_around_attempts = 0;
    ops.max_safe_execution_cost = 1.0;
    ops.plan_only = false;
    ops.replan = false;
    ops.replan_attempts = 0;
    ops.replan_delay = 0.0;
    if (!sendMoveGroupPoseGoal(ops, goal_pose)) {
        return false;
    }

    if (m_result.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        return false;
    } else {
        // TODO: slerp trajectory
        return true;
    }
    return false;
}

bool MoveArmNode::moveToGoalJoints(
    const moveit_msgs::RobotState& start,
    const rcta::MoveArmGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO("move to goal joints");
    return false;
}

} //namespace rcta
