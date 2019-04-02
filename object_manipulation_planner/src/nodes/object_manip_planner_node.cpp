// standard includes
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <gperftools/profiler.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/occupancy_grid.h>
#include <smpl/stl/memory.h>
#include <smpl_moveit_interface/planner/moveit_robot_model.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>

// project includes
#include "assert.h"
#include "object_manip_planner.h"
#include "object_manip_model.h"
#include "object_manip_checker.h"
//#include "roman_robot_model.h"

struct CurrentStateMonitor
{
    std::mutex m;
    std::shared_ptr<tf::TransformListener> listener;
    moveit::core::RobotState curr_state;

    CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model)
        : curr_state(robot_model)
    { }
};

void Init(CurrentStateMonitor* monitor)
{
    monitor->curr_state.setToDefaultValues();
}

void UpdateCurrentState(
    CurrentStateMonitor* monitor,
    const sensor_msgs::JointState& joint_state);

auto GetCurrentState(CurrentStateMonitor* monitor) -> moveit::core::RobotState;

// A necessary evil
namespace std {
template <class Key, class Value>
auto operator<<(std::ostream& o, const std::map<Key, Value>& m) -> std::ostream&
{
    o << '{';
    auto i = 0;
    for (auto& e : m) {
        if (i++ != 0) {
            o << ", ";
        }
        o << '(' << e.first << ", " << e.second << ')';
    }
    o << '}';

    return o;
}
} // namespace

// Downloads a parameter from the parameter server. If the downloaded succeeds,
// log the parameter's value, otherwise log an error message.
template <class T>
bool GetParam(const ros::NodeHandle& nh, const std::string& name, T* value)
{
    if (!nh.getParam(name, *value)) {
        ROS_ERROR("Failed to download param '%s' from the param server", name.c_str());
        return false;
    }

    ROS_INFO_STREAM("Retrieved parameter " << name << " = " << *value);
    return true;
}

// Execute a sequence of trajectory/gripper commands.
bool ExecuteTrajectory(
    const ros::NodeHandle& nh,
    const std::vector<std::unique_ptr<Command>>& commands)
{
    using GripperCommandActionClient =
            actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

    using FollowJointTrajectoryActionClient =
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

    auto traj_client_name = std::string();
    auto gripper_client_name = std::string();
    if (!GetParam(nh, "follow_joint_trajectory_action_name", &traj_client_name) ||
        !GetParam(nh, "gripper_command_action_name", &gripper_client_name))
    {
        return 1;
    }

    ROS_INFO("Wait for action server '%s'", traj_client_name.c_str());
    FollowJointTrajectoryActionClient traj_client(traj_client_name);
    if (!traj_client.waitForServer()) {
        ROS_WARN("Failed to wait for action server '%s'", traj_client_name.c_str());
        return 1;
    }

    ROS_INFO("Wait for GripperCommand action server '%s'", gripper_client_name.c_str());
    GripperCommandActionClient gripper_client(gripper_client_name);
    if (!gripper_client.waitForServer()) {
        ROS_WARN("Failed to wait for action server '%s'", gripper_client_name.c_str());
        return 1;
    }

    // move arm to the pregrasp configuration
    // open the gripper
    // move arm from pre-grasp-to-grasp configuration
    // close the gripper
    // manipulate the object
    // open the gripper
    // move the arm to the post-grasp configuration

    for (auto& command : commands) {
        if (command->type == Command::Type::Gripper) {
            auto* c = static_cast<GripperCommand*>(command.get());
            control_msgs::GripperCommandGoal goal;
            if (c->open) {
                goal.command.position = 0.0841; //1.0;
            } else {
                goal.command.position = 0.0;
            }

            ROS_INFO("%s gripper", c->open ? "Open" : "Close");
            auto res = gripper_client.sendGoalAndWait(goal);
            if (res.state_ == res.SUCCEEDED) {
                ROS_INFO("gripper client returned with state '%s'", res.toString().c_str());
            } else {
                ROS_WARN("gripper client returned with state '%s' (%s)", res.toString().c_str(), res.getText().c_str());
            }
        } else if (command->type == Command::Type::Trajectory) {
            auto* c = static_cast<TrajectoryCommand*>(command.get());

            control_msgs::FollowJointTrajectoryGoal traj;
#if 0
            traj.trajectory.header.stamp = ros::Time(0);
#else
            traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
#endif
            traj.trajectory.header.frame_id = "";

            traj.trajectory.joint_names = {
                "limb_right_joint1",
                "limb_right_joint2",
                "limb_right_joint3",
                "limb_right_joint4",
                "limb_right_joint5",
                "limb_right_joint6",
                "limb_right_joint7",
                "torso_joint1",
            };

            traj.trajectory.points.resize(c->trajectory.getWayPointCount());
            for (auto i = 0; i < c->trajectory.getWayPointCount(); ++i) {
                std::vector<double> positions;
                positions.resize(traj.trajectory.joint_names.size());
                for (auto j = 0; j < traj.trajectory.joint_names.size(); ++j) {
                    auto& joint_name = traj.trajectory.joint_names[j];
                    positions[j] = c->trajectory.getWayPoint(i).getVariablePosition(joint_name);
                }
                traj.trajectory.points[i].positions = std::move(positions);

                traj.trajectory.points[i].time_from_start =
                        ros::Duration(c->trajectory.getWayPointDurationFromStart(i));

                ROS_DEBUG("%zu positions, t(%d) = %f", traj.trajectory.points[i].positions.size(), i, traj.trajectory.points[i].time_from_start.toSec());
            }

            ROS_INFO("Execute trajectory");
            auto res = traj_client.sendGoalAndWait(traj);
            if (res.state_ == res.SUCCEEDED) {
                ROS_INFO("traj client returned with state '%s'", res.toString().c_str());
            } else {
                ROS_WARN("traj client returned with state '%s' (%s)", res.toString().c_str(), res.getText().c_str());
            }
        } else {
            ROS_ERROR("Unrecognized command type");
            return false;
        }
    }

    return true;
}

// Animate a sequence of trajectory/gripper commands. Gripper commands are
// displayed a short pause, with the robot colored in green for an open command
// and red for a close command.
bool AnimateTrajectory(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::vector<std::unique_ptr<Command>>& commands,
    double speedup = 1.0)
{
    auto factor = speedup == 0.0 ? 0.0 : 1.0 / speedup;
    auto last_wp = moveit::core::RobotState(robot_model);
    for (auto& command : commands) {
        if (command->type == Command::Type::Gripper) {
            if (!ros::ok()) break;

            auto* gripper_cmd = static_cast<GripperCommand*>(command.get());

            auto color = std_msgs::ColorRGBA();
            if (gripper_cmd->open) {
                color.g = 0.5f;
                color.a = 0.9f;
            } else {
                color.r = 0.5f;
                color.a = 0.9f;
            }

            auto ma = visualization_msgs::MarkerArray();
            last_wp.getRobotMarkers(
                    ma,
                    last_wp.getRobotModel()->getLinkModelNames(),
                    color,
                    "trajectory",
                    ros::Duration(0));

            SV_SHOW_INFO(ma);

            ros::Duration(factor * 1.0).sleep();
        } else if (command->type == Command::Type::Trajectory) {
            if (!ros::ok()) break;

            auto* trajectory = static_cast<TrajectoryCommand*>(command.get());
            for (auto i = 0; i < trajectory->trajectory.getWayPointCount(); ++i) {
                auto dur = trajectory->trajectory.getWayPointDurations()[i];

                auto& wp = trajectory->trajectory.getWayPoint(i);

                auto color = std_msgs::ColorRGBA();
                color.r = color.g = color.b = 0.5f;
                color.a = 0.9f;

                auto ma = visualization_msgs::MarkerArray();
                wp.getRobotMarkers(
                        ma,
                        wp.getRobotModel()->getLinkModelNames(),
                        color,
                        "trajectory",
                        ros::Duration(0));

                SV_SHOW_INFO(ma);

                ros::Duration(factor * dur).sleep();
            }

            last_wp = trajectory->trajectory.getLastWayPoint();
        } else {
            ROS_ERROR("Unrecognized trajectoy command type");
            return false;
        }
    }
    return true;
}

using ManipulateObjectActionServer =
        actionlib::SimpleActionServer<cmu_manipulation_msgs::ManipulateObjectAction>;

// because fuck you moveit, plz stahp throwing exceptions...unreferenced
// but keeping this around in case we explode
//
// We exploded. Someone please explain to me why we have a return code
// on robotStateMsgToRobotState if we're just going to throw an exception.
// - Andrew on 3/10/2019
bool RobotHasVariable(
    const moveit::core::RobotModel& model,
    const std::string& name)
{
    auto it = std::find(begin(model.getVariableNames()), end(model.getVariableNames()), name);
    return it != end(model.getVariableNames());
};

int GetVariableIndex(moveit::core::RobotState* state, const std::string& name)
{
    auto it = std::find(
            begin(state->getVariableNames()),
            end(state->getVariableNames()),
            name);

    if (it == end(state->getVariableNames())) return -1;

    return std::distance(begin(state->getVariableNames()), it);
}

// A less pissy version of robotStateMsgToRobotState that doesn't complain when
// extra joints show up in the state message. We want this for convenience so
// test scenarios can specify configurations for the entire RoMan when specific
// RoMans lack different components.
void RobotStateMsgToRobotState(
    const moveit_msgs::RobotState* msg,
    moveit::core::RobotState* state)
{
    for (auto i = 0; i < msg->joint_state.name.size(); ++i) {
        auto& joint_name = msg->joint_state.name[i];

        auto index = GetVariableIndex(state, joint_name);
        if (index < 0) continue;

        auto position = msg->joint_state.position[i];
        state->setVariablePosition(index, position);
    }

    for (auto i = 0; i < msg->multi_dof_joint_state.joint_names.size(); ++i) {
        auto& joint_name = msg->multi_dof_joint_state.joint_names[i];

        // explicit check here to avoid stupid error spam
        if (!state->getRobotModel()->hasJointModel(joint_name)) continue;

        auto* joint = state->getRobotModel()->getJointModel(joint_name);
        assert(joint != NULL);

        auto& transform = msg->multi_dof_joint_state.transforms[i];

        auto T_eigen = Eigen::Affine3d{ };
        tf::transformMsgToEigen(transform, T_eigen);

        state->setJointPositions(joint, T_eigen);
    }
}

// Plan and execute (or animate) a sequence of trajectory/gripper commands to
// manipulate the state of an object.
// \param robot_model
// \param group_name needed to create final trajectory
bool ManipulateObject(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const smpl::urdf::RobotModel* object_model,
    smpl::collision::CollisionSpace* cspace,
    sbpl_interface::MoveItRobotModel* planning_model,
    ObjectManipPlanner* planner,
    const moveit::core::RobotState* real_start_state,
    const ros::Publisher& display_publisher,
    const ros::NodeHandle& ph,
    const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
{
    ///////////////////////////////////////
    // Visualize the state of the object //
    ///////////////////////////////////////

    auto* obj_root_joint = GetRootJoint(object_model);
    if (obj_root_joint->type == smpl::urdf::JointType::Floating) {
        auto obj_state = smpl::urdf::RobotState();
        if (InitRobotState(&obj_state, object_model, false, false)) {
            double positions[7];
            positions[0] = msg->object_pose.position.x;
            positions[1] = msg->object_pose.position.y;
            positions[2] = msg->object_pose.position.z;
            positions[3] = msg->object_pose.orientation.x; // blech
            positions[4] = msg->object_pose.orientation.y; // blech
            positions[5] = msg->object_pose.orientation.z; // blech
            positions[6] = msg->object_pose.orientation.w; // blech
            SetJointPositions(&obj_state, obj_root_joint, positions);
            UpdateVisualBodyTransforms(&obj_state);
            SV_SHOW_INFO_NAMED(
                    "object_state",
                    MakeRobotVisualization(
                            &obj_state,
                            smpl::visual::Color{ 1.0f, 0.5f, 0.0f, 1.0f },
                            "map", // TODO: @planning frame
                            "object_state"));
        } else {
            ROS_ERROR("failed to initialize object state?");
        }
    } else {
        ROS_ERROR("expected object model to have a floating joint at the root");
    }

    ///////////////////////////////////////////////////////////////////////////
    // Initialize the start state from the current state and apply overrides //
    ///////////////////////////////////////////////////////////////////////////

    auto start_state = moveit::core::RobotState(*real_start_state);

    RobotStateMsgToRobotState(&msg->start_state, &start_state);

    ///////////////////////////////
    // Visualize the start state //
    ///////////////////////////////

    {
        auto ma = visualization_msgs::MarkerArray();
        auto color = std_msgs::ColorRGBA();
        color.r = 0.8f;
        color.g = 0.8f;
        color.b = 1.0f;
        color.a = 0.9f;
        start_state.getRobotMarkers(
                ma,
                start_state.getRobotModel()->getLinkModelNames(),
                color,
                "start_state",
                ros::Duration(0.0));
        ROS_INFO("Visualize %zu markers", ma.markers.size());
        SV_SHOW_INFO(ma);
    }

    ///////////////////////////////
    // Update the Planning Scene //
    ///////////////////////////////

    ROS_INFO("Update reference state in collision model to the start state");

    for (auto i = 0; i < robot_model->getVariableCount(); ++i) {
        auto& name = robot_model->getVariableNames()[i];
        auto pos = start_state.getVariablePositions()[i];
        if (!cspace->setJointPosition(name, pos)) {
            ROS_ERROR("Failed to set position of joint '%s' to %f in the collision model", name.c_str(), pos);
            return false;
        }
    }

    ROS_INFO("Update reference state in planning model to the start state");

    if (!planning_model->updateReferenceState(start_state)) {
        ROS_ERROR("Failed to update the planning model reference state");
        return false;
    }

    ////////////////////////////
    // Finally, plan the path //
    ////////////////////////////

    ROS_INFO("Plan path!");

    auto object_pose = Eigen::Affine3d();
    tf::poseMsgToEigen(msg->object_pose, object_pose);

    auto object_start_state = msg->object_start;
    auto object_goal_state = msg->object_goal;

    auto allowed_time = msg->allowed_planning_time;

    auto commands = std::vector<std::unique_ptr<Command>>();

    ProfilerStart("omp");
    if (!PlanPath(
            planner,
            start_state,
            object_pose,
            object_start_state,
            object_goal_state,
            allowed_time,
            &commands))
    {
        ProfilerStop();
        ROS_ERROR("Failed to plan path");
        return false;
    }
    ProfilerStop();

    //////////////////////////////
    // display the planned path //
    //////////////////////////////

    if (false) {
        auto trajectory = robot_trajectory::RobotTrajectory(robot_model, group_name);
        MakeRobotTrajectory(&commands, &trajectory);

        ROS_INFO("Display trajectory");
        moveit_msgs::DisplayTrajectory display;

        display.model_id = robot_model->getName();

        display.trajectory.resize(1);
        trajectory.getRobotTrajectoryMsg(display.trajectory[0]);

        moveit::core::robotStateToRobotStateMsg(
                trajectory.getFirstWayPoint(),
                display.trajectory_start);

        display_publisher.publish(display);
    }

    /////////////////////////////////////////////
    // Execute (or animate) the resulting plan //
    /////////////////////////////////////////////

    auto execute = !msg->plan_only;

    if (execute) {
        ExecuteTrajectory(ph, commands);
    } else {
        AnimateTrajectory(robot_model, commands, 5.0);
    }

    ROS_INFO("Crate manipulation successful");
    return true;
}

void UpdateCurrentState(
    CurrentStateMonitor* monitor,
    const sensor_msgs::JointState& joint_state)
{
    std::unique_lock<std::mutex> lock(monitor->m);
    for (auto i = 0; i < joint_state.name.size(); ++i) {
        // skip unknown variable name
        auto& name = joint_state.name[i];
        auto it = std::find(
                begin(monitor->curr_state.getVariableNames()),
                end(monitor->curr_state.getVariableNames()),
                name);
        if (it == end(monitor->curr_state.getVariableNames())) continue;

        auto index = std::distance(begin(monitor->curr_state.getVariableNames()), it);
        auto position = joint_state.position[i];
        monitor->curr_state.setVariablePosition(index, position);
    }
}

auto GetCurrentState(CurrentStateMonitor* monitor) -> moveit::core::RobotState
{
    auto& root_link_name = monitor->curr_state.getRobotModel()->getRootLink()->getName();
    auto T_world_robot = Eigen::Affine3d(Eigen::Affine3d::Identity());
    auto world_frame = "map";
    try {
        auto tf = tf::StampedTransform();
        monitor->listener->lookupTransform(world_frame, root_link_name, ros::Time(0), tf);
        tf::transformTFToEigen(tf, T_world_robot);
    } catch (const tf::TransformException& ex) {
        // TODO:: This isn't exactly what we need here, since there is no state
        // information for from the world frame to the root link of our robot,
        // which for the current URDF is the fake "z" link, and not the actual
        // root link of the robot. Luckily, all of our examples right now
        // don't move the robot and we have no collision information registered
        // in the global frame
        ROS_WARN("Failed to lookup transform from '%s' to '%s'. Defaulting to identity", world_frame, root_link_name.c_str());
    }
    auto lock = std::unique_lock<std::mutex>(monitor->m);
    auto state = monitor->curr_state;
    state.setJointPositions(state.getRobotModel()->getRootJoint()->getName(), T_world_robot);
    return state;
}

// 1. Generate a trajectory using the model of the object
// 2. Save the example trajectory as a demonstration
// 3. Construct state space: (base/x, base/y, base/theta, torso, ee/x, ee/y,
//    ee/z, ee/yaw, arm/free_angle)
// 4. Construct a Workspace Lattice with a special action set.
//  * The original action space, replicated for all values of z
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_manip_planner");
    auto nh = ros::NodeHandle();
    auto ph = ros::NodeHandle("~");

    auto display_publisher =
            ph.advertise<moveit_msgs::DisplayTrajectory>("planned_path", 1);

    auto visualizer = smpl::VisualizerROS();
    smpl::visual::set_visualizer(&visualizer);

    ros::Duration(0.5).sleep(); // let publisher set up

    auto group_name = std::string();
    auto tip_link = std::string();
    auto ik_group_name = std::string();
    if (!GetParam(ph, "group_name", &group_name) ||
        !GetParam(ph, "tip_link", &tip_link) ||
        !GetParam(ph, "ik_group_name", &ik_group_name))
    {
        return 1;
    }

    /////////////////////////////////////////////
    // Load the Robot Model from the URDF/SRDF //
    /////////////////////////////////////////////

    ROS_INFO("Load Robot Model");

    // get the nominal urdf/srdf
    auto loader = robot_model_loader::RobotModelLoader();
    auto robot_model = loader.getModel();
    if (robot_model == NULL) {
        ROS_ERROR("Failed to load Robot Model");
        return 1;
    }

    auto redundant_joints = std::vector<std::string>();
    if (ik_group_name == "right_arm") {
        redundant_joints = { "limb_right_joint3" };
    } else if (ik_group_name == "right_arm_and_torso") {
        redundant_joints = { "torso_joint1", "limb_right_joint3" };
    } else {
        ROS_ERROR("Get better at defining IK groups");
        return 1;
    }

    auto* ik_group = robot_model->getJointModelGroup(ik_group_name);
    if (!ik_group->setRedundantJoints(redundant_joints)) {
        ROS_ERROR("Failed to set redundant joints");
        return 1;
    }

    /////////////////////////////////////////////////////////
    // Load the object model from URDF (for visualization) //
    /////////////////////////////////////////////////////////

    auto obj_description_key = "object_description";
    auto obj_description_abs_key = std::string();
    if (!ph.searchParam(obj_description_key, obj_description_abs_key)) {
        ROS_ERROR("Failed to find 'object_description' on the param server");
        return 1;
    }

    auto object_description = std::string();

    if (!ph.getParam(obj_description_abs_key, object_description)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", obj_description_abs_key.c_str());
        return 1;
    }

    auto object_urdf = urdf::parseURDF(object_description);
    if (object_urdf == NULL) {
        ROS_ERROR("Failed to parse object URDF");
        return 1;
    }

    auto object_model = smpl::urdf::RobotModel();
    auto j_object_world = smpl::urdf::JointSpec();
    j_object_world.origin = smpl::Affine3::Identity();
    j_object_world.axis = smpl::Vector3::Zero();
    j_object_world.name = "world_joint";
    j_object_world.type = smpl::urdf::JointType::Floating;
    if (!InitRobotModel(&object_model, object_urdf.get())) {
        ROS_ERROR("Failed to initialize object model");
        return 1;
    }

    ///////////////////////////////////
    // Initialize the Planning Model //
    ///////////////////////////////////

    ROS_INFO("Initialize Planning Model");

    // want to include the object degree-of-freedom as a free variable in the
    // robot model to be able to use workspace lattice directly
    sbpl_interface::MoveItRobotModel planning_model;
    if (!planning_model.init(robot_model, group_name, ik_group_name)) {
        ROS_ERROR("Failed to initialize robot model");
        return 1;
    }

    ROS_INFO("Parent planning joints:");
    for (auto i = 0; i < planning_model.jointVariableCount(); ++i) {
        ROS_INFO("  %s", planning_model.getPlanningJoints()[i].c_str());
    }

    if (!planning_model.setPlanningLink(tip_link)) {
        ROS_ERROR("Failed to set planning link");
        return 1;
    }

#if 0
    // Create a wrapper around MoveItRobotModel so we can add an additional
    // degree-of-freedom for the z position.
    auto roman_model = RomanRobotModel();
    if (!Init(&roman_model, &planning_model)) {
        ROS_ERROR("Failed to initialize Roman Robot Model");
        return 1;
    }
#endif

    auto omanip = ObjectManipModel();
    auto object_min_position = 0.0;
    auto object_max_position = 1.0;
    auto object_variable_name = "hinge";
    if (!Init(
            &omanip,
#if 0
            &roman_model,
#else
            &planning_model,
#endif
            object_variable_name,
            object_min_position,
            object_max_position))
    {
        ROS_ERROR("Failed to initialize Object Manipulation Model");
        return 1;
    }

    ROS_INFO_STREAM("Planning Model Variables: " << omanip.getPlanningJoints());

    //////////////////////////////////////
    // Initialize the Collision Checker //
    //////////////////////////////////////

    ROS_INFO("Initialize Collision Checker");

    // frame of the planning/collision model, for visualization
    auto planning_frame = std::string();
    ph.param<std::string>("planning_frame", planning_frame, "map");

    double size_x, size_y, size_z, origin_x, origin_y, origin_z, resolution, max_dist;
    auto gh = ros::NodeHandle(ph, "grid");
    if (!GetParam(gh, "size_x", &size_x) ||
        !GetParam(gh, "size_y", &size_y) ||
        !GetParam(gh, "size_z", &size_z) ||
        !GetParam(gh, "origin_x", &origin_x) ||
        !GetParam(gh, "origin_y", &origin_y) ||
        !GetParam(gh, "origin_z", &origin_z) ||
        !GetParam(gh, "resolution", &resolution) ||
        !GetParam(gh, "max_dist", &max_dist))
    {
        return 1;
    }

    auto df = std::make_shared<smpl::EuclidDistanceMap>(origin_x, origin_y, origin_z, size_x, size_y, size_z, resolution, max_dist);
    auto grid = smpl::OccupancyGrid(df, false);

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    auto config = smpl::collision::CollisionModelConfig();
    if (!smpl::collision::CollisionModelConfig::Load(ph, config)) {
        ROS_ERROR("Failed to load Collision Model Configuration");
        return 1;
    }

    // NOTE: Initialize CollisionSpace with the planning joints of the parent
    // (non-object) model, since the CollisionSpace has no knowledge of the
    // object joint. Also, it's pure luck that the CollisionSpace ignores "too
    // large" vectors.
    auto cspace = smpl::collision::CollisionSpace();
#if 1
    if (!cspace.init(
            &grid,
            *robot_model->getURDF(),
            config,
            group_name,
            omanip.parent_model->getPlanningJoints()))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }
#else
    if (!cspace.init(
            &grid,
            *urdf_copy,
            config,
            group_name,
            omanip.parent_model->getPlanningJoints()))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }
#endif

    smpl::collision::CollisionObject ground_object;
    ground_object.id = "ground";

    smpl::collision::PlaneShape ground_shape;
    ground_shape.a = ground_shape.b = 0.0;
    ground_shape.c = 1.0;
    ground_shape.d = 0.13613; // annoyingly not 0, because the root frame of the RoMan is not on the ground.

    ground_object.shapes.push_back(&ground_shape);
    ground_object.shape_poses.push_back(Eigen::Affine3d::Identity());

    cspace.insertObject(&ground_object);

    SV_SHOW_INFO(cspace.getCollisionWorldVisualization());

    ObjectManipChecker ochecker;
    if (!Init(&ochecker, &cspace)) {
        return false;
    }

    ////////////////////////////
    // Initialize the Planner //
    ////////////////////////////

    ROS_INFO("Initialize Object Manipulation Planner");

    auto params = ObjectManipPlannerParams();
    {
        ph.param("use_rotation", params.use_rotation, true);
        ph.param("disc_rotation_heuristic", params.disc_rotation_heuristic, true);
        ph.param("rot_db", params.rot_db, smpl::to_radians(2));
        int heading_condition;
        ph.param("heading_condition", heading_condition, (int)ObjectManipPlannerParams::HeadingCondition::Discrete);
        params.heading_condition = (ObjectManipPlannerParams::HeadingCondition)heading_condition;
        ph.param("heading_thresh", params.heading_thresh, 0.05); // 0.35, 0.1, 0.15
        ph.param("disc_position_heuristic", params.disc_position_heuristic, true);
        ph.param("pos_db", params.pos_db, 0.1);
        ph.param("rot_weight", params.rot_weight, 0.45 / smpl::to_radians(45));
        ph.param("base_weight", params.base_weight, 10.0);
        int combination;
        ph.param("combination", combination, (int)ObjectManipPlannerParams::CombinationMethod::Max);
        params.combination = (ObjectManipPlannerParams::CombinationMethod)combination;
        ph.param("w_egraph", params.w_egraph, 5.0);
        ph.param("w_heuristic", params.w_heuristic, 100.0);
    }

    ObjectManipPlanner planner;
    if (!Init(&planner, &omanip, &ochecker, &grid, &params)) {
        ROS_ERROR("Failed to initialize Object Manipulation Planner");
        return 1;
    }

    auto demos = std::string();
    if (!GetParam(ph, "demonstrations_path", &demos)) {
        return 1;
    }

    ROS_INFO("Load demonstrations");
    if (!LoadDemonstrations(&planner, demos)) {
        return 1;
    }

    ////////////////////////////////////////
    // Fire up planning/execution service //
    ////////////////////////////////////////

    CurrentStateMonitor state_monitor(robot_model);
    Init(&state_monitor);
    state_monitor.listener = std::make_shared<tf::TransformListener>();

    using JointStateCallback = boost::function<void(const sensor_msgs::JointState::ConstPtr&)>;
    JointStateCallback jsfun =
    [&](const sensor_msgs::JointState::ConstPtr& msg)
    {
        UpdateCurrentState(&state_monitor, *msg);
    };

    auto sub = nh.subscribe("joint_states", 10, jsfun);

    auto autostart = false;
    ManipulateObjectActionServer server(
            "manipulate_object",
            [&](const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
            {
                auto curr_state = GetCurrentState(&state_monitor);
                if (ManipulateObject(
                        robot_model,
                        group_name,
                        &object_model,
                        &cspace,
                        &planning_model,
                        &planner,
                        &curr_state,
                        display_publisher,
                        ph,
                        msg))
                {
                    server.setSucceeded();
                } else {
                    server.setAborted();
                }
            },
            autostart);

    ROS_INFO("Start action server");
    server.start();
    ros::spin();

    return 0;
}

