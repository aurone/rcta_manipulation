#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/console/nonstd.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <urdf_parser/urdf_parser.h>

static
auto to_cstring(moveit_msgs::MoveItErrorCodes code) -> const char*
{
    switch (code.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
        return "FAILURE";

    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";

    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";

    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";

    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME";

    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE";

    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";

    default:
        return "UNRECOGNIZED";
    }
}

using ManipulateObjectActionServer =
        actionlib::SimpleActionServer<cmu_manipulation_msgs::ManipulateObjectAction>;

template <class T>
bool GetParam(const ros::NodeHandle& nh, const std::string& key, T& value)
{
    if (!nh.getParam(key, value)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", key.c_str());
        return false;
    }

    return true;
}

using GripperCommandActionClient =
        actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

template <class Range>
void reverse(Range&& range)
{
    std::reverse(begin(range), end(range));
}

bool OpenGripper(GripperCommandActionClient* gripper_client)
{
    ROS_INFO("Open gripper");
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.0841;
    auto res = gripper_client->sendGoalAndWait(gripper_goal);
    return res.state_ == res.SUCCEEDED;
}

bool CloseGripper(GripperCommandActionClient* gripper_client)
{
    ROS_INFO("Close gripper");
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.0;
    auto res = gripper_client->sendGoalAndWait(gripper_goal);
    return res.state_ == res.SUCCEEDED;
}

template <class T>
auto interp(const T& src, const T& dst, double t) -> T
{
    return (1.0 - t) * src + t * dst;
}

bool MoveCartesian(
    moveit::planning_interface::MoveGroupInterface* move_group,
    const Eigen::Affine3d& dst_pose,
    const std::string& group_name,
    const std::string& link_name,
    double pct_thresh = 1.0)
{
    auto src_pose = move_group->getCurrentState()->getGlobalLinkTransform(link_name);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.resize(1);
    tf::poseEigenToMsg(dst_pose, waypoints[0]);
    auto eef_step = 0.02;
    auto jump_thresh = 10.0;
    auto traj = moveit_msgs::RobotTrajectory();
    auto avoid_collisions = true;
    auto err = moveit::planning_interface::MoveItErrorCode();
    auto pct = move_group->computeCartesianPath(
            waypoints, eef_step, jump_thresh, traj, avoid_collisions, &err);
    if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to plan cartesian-style!");
        return false;
    }

    ROS_INFO("Cartesian move at %f%%", pct);

    if (pct < pct_thresh) {
        ROS_ERROR("Failed to compute cartesian motion (target percent = %f, achieved percent = %f)", pct_thresh, pct);
        return false;
    }

    auto curr_state = move_group->getCurrentState();
    if (curr_state == NULL) {
        ROS_INFO("Failed to retrieve current state");
        return false;
    }

    ///////////////////////
    // Create trajectory //
    ///////////////////////

    auto ugh_traj = robot_trajectory::RobotTrajectory(
            move_group->getRobotModel(), group_name);
    ugh_traj.setRobotTrajectoryMsg(*curr_state, traj);

    trajectory_processing::IterativeParabolicTimeParameterization itp;
    if (!itp.computeTimeStamps(ugh_traj)) {
        ROS_ERROR("Failed to compute timestamps");
        return false;
    }

    ROS_INFO("computed time stamps");

    /////////////////////
    // Convert to Plan //
    /////////////////////

    auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
    robotStateToRobotStateMsg(*curr_state, plan.start_state_);
    ugh_traj.getRobotTrajectoryMsg(plan.trajectory_);

    for (auto& point : plan.trajectory_.joint_trajectory.points) {
        ROS_INFO_STREAM(point.time_from_start.toSec() << ": " << point.positions);
    }

    for (auto& point : plan.trajectory_.joint_trajectory.points) {
        point.time_from_start += ros::Duration(0.5);
    }

    ////////////////////
    // Execute motion //
    ////////////////////

    err = move_group->execute(plan);
    if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("Failed to move cartesian-style (%s)!", to_cstring(err));
        return false;
    }

    return true;
}

bool MoveRelativeCartesian(
    moveit::planning_interface::MoveGroupInterface* move_group,
    const Eigen::Affine3d& dst_pose,
    const std::string& group_name,
    const std::string& link_name,
    double pct_thresh = 1.0)
{
    auto src_pose = move_group->getCurrentState()->getGlobalLinkTransform(link_name);
    auto tgt_pose = src_pose * dst_pose;
    return MoveCartesian(move_group, tgt_pose, group_name, link_name, pct_thresh);
}

template <class Sampler>
bool PlanManipulationTrajectory(
    moveit::planning_interface::MoveGroupInterface* move_group,
    const std::string& group_name,
    const std::string& tool_link_name,
    int samples,
    Sampler sampler,
    moveit::planning_interface::MoveGroupInterface::Plan* oplan)
{
    auto interm_state = *move_group->getCurrentState();
    auto manip_traj = robot_trajectory::RobotTrajectory(interm_state.getRobotModel(), group_name);
    auto ids = (int32_t)0;

    for (auto i = 0; i < samples; ++i) {
        auto alpha = (double)i / (double)(samples - 1);
        auto contact_pose = sampler(alpha);

        auto robot_model = interm_state.getRobotModel();
        auto* group = robot_model->getJointModelGroup(group_name);

        auto consistency_limits = std::vector<double>(group->getVariableCount(), smpl::to_radians(22.5));

        if (interm_state.setFromIK(group, contact_pose, tool_link_name, consistency_limits)) {
            visualization_msgs::MarkerArray ma;
            std_msgs::ColorRGBA color;
            color.r = 1.0f;
            color.g = 0.5f;
            color.b = 0.0f;
            color.a = 0.8f;
            interm_state.getRobotMarkers(ma, robot_model->getLinkModelNames(), color, "constrained", ros::Duration(0));
            for (auto& m : ma.markers) {
                m.id = ids++;
            }
            SV_SHOW_INFO(ma);
            manip_traj.addSuffixWayPoint(interm_state, 1.0);
        }
    }

    // timestamp waypoint
    trajectory_processing::IterativeParabolicTimeParameterization itp;
    if (!itp.computeTimeStamps(manip_traj)) {
        ROS_ERROR("Failed to compute timestamps");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robotStateToRobotStateMsg(*move_group->getCurrentState(), plan.start_state_);
    manip_traj.getRobotTrajectoryMsg(plan.trajectory_);

    *oplan = std::move(plan);
    return true;
}

bool ManipulateObject(
    const cmu_manipulation_msgs::ManipulateObjectGoal* goal,
    moveit::planning_interface::MoveGroupInterface* move_group,
    const std::string* group_name,
    GripperCommandActionClient* gripper_client,
    const smpl::urdf::RobotModel* object_model)
{
    ROS_INFO("Received manipulate object request");
    ROS_INFO("  start_state");
    ROS_INFO("  object_id: %s", goal->object_id.c_str());
    ROS_INFO("  object_pose: (%f, %f, %f, %f, %f, %f, %f)",
            goal->object_pose.position.x,
            goal->object_pose.position.y,
            goal->object_pose.position.z,
            goal->object_pose.orientation.w,
            goal->object_pose.orientation.x,
            goal->object_pose.orientation.y,
            goal->object_pose.orientation.z);
    ROS_INFO("  object_start: %f", goal->object_start);
    ROS_INFO("  object_goal: %f", goal->object_goal);
    ROS_INFO("  allowed planning time: %f", goal->allowed_planning_time);
    ROS_INFO("  plan_only: %d", (int)goal->plan_only);

    auto curr_state = move_group->getCurrentState();
    if (curr_state == NULL) {
        ROS_ERROR("No current state");
        return false;
    }

    auto start_state = *curr_state;

    //////////////////////////
    // Determine grasp pose //
    //////////////////////////

    auto object_state = smpl::urdf::RobotState();
    if (!InitRobotState(&object_state, object_model, false, false)) {
        return false;
    }

    auto object_pose =
            smpl::Affine3(smpl::Translation3(
                    goal->object_pose.position.x,
                    goal->object_pose.position.y,
                    goal->object_pose.position.z) *
            smpl::Quaternion(
                    goal->object_pose.orientation.w,
                    goal->object_pose.orientation.x,
                    goal->object_pose.orientation.y,
                    goal->object_pose.orientation.z));

    auto* lid_var = GetVariable(object_model, "lid_joint");
    auto* handle_var = GetVariable(object_model, "handle_joint");
    auto* root_joint = GetRootJoint(object_model);

    if (lid_var == NULL || handle_var == NULL || root_joint == NULL) {
        ROS_ERROR("Missing some variables/joints or something");
        return false;
    }

    auto* lid_limits = GetVariableLimits(lid_var);
    if (lid_limits == NULL) return false;

    auto span = lid_limits->max_position - lid_limits->min_position;

    auto get_lid_pos_from_pct = [&](double pct) {
        return (1.0 - pct) * lid_limits->min_position + pct * lid_limits->max_position;
    };

    SetVariablePosition(&object_state, lid_var, get_lid_pos_from_pct(goal->object_start));
    SetVariablePosition(&object_state, handle_var, 0.0);
    SetJointPositions(&object_state, root_joint, &object_pose);

    auto* contact_link = GetLink(object_model, "tool");
    if (contact_link == NULL) {
        ROS_ERROR("No 'tool' link");
        return false;
    }

    auto contact_pose = *GetUpdatedLinkTransform(&object_state, contact_link);

    auto print_pose = [](const char* prefix, const smpl::Affine3& pose)
    {
        double y, p, r;
        smpl::get_euler_zyx(pose.rotation(), y, p, r);
        ROS_INFO("%s = (%f, %f, %f, %f, %f, %f)",
                prefix,
                pose.translation().x(),
                pose.translation().y(),
                pose.translation().z(),
                y, p, r);
    };

    print_pose("contact link pose", contact_pose);

    // rotate so that the end effector is correct
    contact_pose *= smpl::AngleAxis(0.5 * M_PI, smpl::Vector3::UnitX());

    print_pose("grasp pose", contact_pose);

    // pre-grasp = straight back by some offset
    auto pregrasp_offset = -0.25;
    auto pregrasp_pose = smpl::Affine3(
            contact_pose *
            smpl::Translation3(pregrasp_offset, 0.0, 0.0));

    print_pose("pregrasp pose", pregrasp_pose);

    auto tool_link_name = "limb_right_tool0";

    move_group->setEndEffectorLink(tool_link_name);

    ////////////////////////////
    // Move to pre-grasp pose //
    ////////////////////////////

    {
        move_group->setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]");
        move_group->setGoalPositionTolerance(0.02);
        move_group->setGoalOrientationTolerance(smpl::to_radians(2.0));

        move_group->setPoseTarget(pregrasp_pose, tool_link_name);
        auto err = move_group->move();
    }

    //////////////////////
    // open the gripper //
    //////////////////////

    if (!OpenGripper(gripper_client)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }

    ///////////////////
    // move to grasp //
    ///////////////////

    if (!MoveCartesian(move_group, contact_pose, *group_name, tool_link_name, 0.9)) {
        return false;
    }

    ///////////////////////
    // close the gripper //
    ///////////////////////

    if (!CloseGripper(gripper_client)) {
        ROS_ERROR("Failed to close gripper");
        return false;
    }

    ///////////////////////////
    // manipulate the object //
    ///////////////////////////

    {
        auto sample_manifold = [&](double alpha) -> Eigen::Affine3d
        {
            auto* contact_link = GetLink(object_model, "tool");
            if (contact_link == NULL) {
                ROS_ERROR("No 'tool' link");
                return Eigen::Affine3d::Identity();
            }

            SetVariablePosition(&object_state, lid_var, lid_limits->max_position);
            auto default_pose = *GetUpdatedLinkTransform(&object_state, contact_link);

            auto s = interp(
                    get_lid_pos_from_pct(goal->object_start),
                    get_lid_pos_from_pct(goal->object_goal),
                    alpha);
            ROS_DEBUG("set lid to %f", s);

            SetVariablePosition(&object_state, lid_var, s);
            auto contact_pose = *GetUpdatedLinkTransform(&object_state, contact_link);
            contact_pose = smpl::Translation3(contact_pose.translation()) *
                    smpl::Quaternion(default_pose.rotation());
            contact_pose *= smpl::AngleAxis(0.5 * M_PI, smpl::Vector3::UnitX());
            return contact_pose;
        };

        auto theta = std::fabs(
                get_lid_pos_from_pct(goal->object_goal) -
                get_lid_pos_from_pct(goal->object_start));
        auto radius = 0.15;
        auto arc = theta * radius; // s = r*theta;
        ROS_INFO("arc = %f", arc);

        auto res = 0.01; // centimeters
        auto samples = std::max(2, (int)std::round(arc / res));

        auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
        if (!PlanManipulationTrajectory(
                move_group,
                *group_name, tool_link_name,
                samples, sample_manifold,
                &plan))
        {
            return false;
        }

        auto err = move_group->execute(plan);
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to execute manip trajectory");
            return false;
        }
    }

    ////////////////////////////
    // open the gripper again //
    ////////////////////////////

    if (!OpenGripper(gripper_client)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }

    //////////////////////////////
    // move to pre-release pose //
    //////////////////////////////

    {
        auto curr_state = *move_group->getCurrentState();
        auto& tool_transform = curr_state.getGlobalLinkTransform(tool_link_name);
        auto prerelease_pose = tool_transform * Eigen::Translation3d(-0.15, 0.0, 0.0);
        MoveCartesian(move_group, prerelease_pose, *group_name, tool_link_name, 0.0);
    }

    //////////////////////////////////
    // move back to the start state //
    //////////////////////////////////

    {
        move_group->setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_JD_ML]");
        move_group->setGoalJointTolerance(smpl::to_radians(1.0));
        move_group->setJointValueTarget(start_state);
        auto err = move_group->move();
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return false;
        }
    }

    return true;
}

// A simple program that accomplishes the crate opening task using the following
// sequence of actions
// 1. Determine a feasible pre-grasp and grasp pose for grasping the crate handle
// 2. Execute a feasible motion to move the manipulator to move the end effector
//    to the pre-grasp pose
// 3. Open the gripper
// 4. Execute a feasible cartesian motion to move the manipulator from the
//    pre-grasp pose to the grasp pose
// 5. Close the gripper
// 6. Execute a motion to move the manipulator to open the crate
// 7. Open the gripper
// 8. Return to a known pre-manipulation configuration or the start state
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_manip_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);

    /////////////////////
    // load the object //
    /////////////////////

    auto object_description = std::string();
    if (!GetParam(nh, "object_description", object_description)) return 1;

    auto object_urdf = urdf::parseURDF(object_description);
    if (object_urdf == NULL) {
        ROS_ERROR("Failed to parse object URDF");
        return 1;
    }

    auto object_model = smpl::urdf::RobotModel();
    smpl::urdf::JointSpec world_joint;
    world_joint.origin = smpl::Affine3::Identity();
    world_joint.axis = smpl::Vector3::Zero();
    world_joint.name = "world_joint";
    world_joint.type = smpl::urdf::JointType::Floating;
    if (!InitRobotModel(&object_model, object_urdf.get())) {
        ROS_ERROR("Failed to initialize object model");
        return 1;
    }

    /////////////////////////////////////
    // Initialize Move Group Interface //
    /////////////////////////////////////

    auto group_name = std::string("right_arm_and_torso");
    auto move_group =
            moveit::planning_interface::MoveGroupInterface(group_name);

    // TODO: ugh of course this is only settable in the planning frame
    move_group.setWorkspace(-0.5, -1.5, -0.2, 1.5, 1.5, 1.8);

    auto gripper_client_name = std::string("rcta_right_robotiq_controller/gripper_action");

    ROS_INFO("Wait for GripperCommand action server '%s'", gripper_client_name.c_str());
    GripperCommandActionClient gripper_client(gripper_client_name);
    if (!gripper_client.waitForServer()) {
        ROS_WARN("Failed to wait for action server '%s'", gripper_client_name.c_str());
        return 1;
    }

    ////////////////
    // Node Stuff //
    ////////////////

    auto spinner = ros::AsyncSpinner(2);

    auto autostart = false;
    ManipulateObjectActionServer server(
            "manipulate_object",
            [&](const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
            {
                if (!ManipulateObject(msg.get(), &move_group, &group_name, &gripper_client, &object_model)) {
                    ROS_ERROR("Action server aborted");
                    server.setAborted();
                } else {
                    ROS_INFO("Action succeeded");
                    server.setSucceeded();
                }
            },
            autostart);

    server.start();
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
