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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetStateValidity.h>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
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
#include <smpl_urdf_robot_model/robot_model.h>

// project includes
#include "assert.h"
#include "object_manip_planner.h"
#include "object_manip_model.h"
#include "object_manip_checker.h"
//#include "roman_robot_model.h"

//for sleeping
#include <chrono>
#include <thread>

using GripperCommandActionClient =
        actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

//using GripperCommandActionClient =
//        actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

using FollowJointTrajectoryActionClient =
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

struct CurrentStateMonitor
{
    std::mutex m;
    std::shared_ptr<tf::TransformListener> listener;
    moveit::core::RobotState curr_state;

    CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model)
        : curr_state(robot_model)
    { }

#if 0
    CurrentStateMonitor(const CurrentStateMonitor&) = delete;
    CurrentStateMonitor(CurrentStateMonitor&&) = default;

    auto operator=(const CurrentStateMonitor&) -> CurrentStateMonitor& = delete;
    auto operator=(CurrentStateMonitor&&) -> CurrentStateMonitor& = default;
#endif
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

bool AdjustGripper()
{
    // init move_group
    // adjust the gripper position if necessary
    // check again maybe ?
    // return true

    auto group_name = std::string("right_arm_and_torso");
    auto move_group =
            moveit::planning_interface::MoveGroupInterface(group_name);

    move_group.setPlanningTime(10.0);

    move_group.setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]");
    move_group.setGoalPositionTolerance(0.02);
    move_group.setGoalOrientationTolerance(smpl::to_radians(2.0));

    move_group.setWorkspace(-0.5, -1.5, -0.2, 1.5, 1.5, 1.8);

    auto gripper_client_name = std::string("rcta_right_robotiq_controller/gripper_action");

    auto tool_link_name = "limb_right_tool0";

    ROS_INFO("Wait for GripperCommand action server '%s'", gripper_client_name.c_str());
    GripperCommandActionClient gripper_client(gripper_client_name);
    if (!gripper_client.waitForServer()) {
        ROS_WARN("Failed to wait for action server '%s'", gripper_client_name.c_str());
        return 1;
    }

    ROS_INFO("finished waiting for the GripperCommand action server. out now ");

    move_group.setEndEffectorLink(tool_link_name);

    {
        auto curr_state = *move_group.getCurrentState();
        auto& tool_transform = curr_state.getGlobalLinkTransform(tool_link_name);
        // TODO - change this value
        auto move_pose =
                tool_transform *
                Eigen::Translation3d(0.0, -0.1, 0.0);

        move_group.setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]");
        move_group.setGoalPositionTolerance(0.02);
        move_group.setGoalOrientationTolerance(smpl::to_radians(2.0));
        move_group.setPoseTarget(move_pose, tool_link_name);
        auto err = move_group.move();
        std::cout << "moved in adjust gripper " << std::endl;
    }

    return true;
}

bool CheckGripperGrip()
{
    // subscribe to the joint_space topic
    // read the gripper's current position
    // return feedback

    auto nh = ros::NodeHandle();

    auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "joint_states", nh, ros::Duration(10.0));

    int size_msg = msg->position.size();

    if (msg) {
        ROS_INFO("position of finger (%d th) is %d", 1, msg->position[size_msg - 6]);
        ROS_INFO("position of finger (%d th) is %d", 2, msg->position[size_msg - 5]);
        ROS_INFO("position of finger (%d th) is %d", 3, msg->position[size_msg - 4]);
    }

    return 0;
}

// Execute a sequence of trajectory/gripper commands.
// move arm to the pregrasp configuration
// open the gripper
// move arm from pre-grasp-to-grasp configuration
// close the gripper
// manipulate the object
// open the gripper
// move the arm to the post-grasp configuration
bool ExecuteTrajectory(
    FollowJointTrajectoryActionClient* traj_client,
    GripperCommandActionClient* gripper_client,
    ros::Publisher* gripper_command_pub,
    const std::vector<std::unique_ptr<Command>>& commands)
{
    ROS_INFO("Wait for FollowJointTrajectory action server");
    if (!traj_client->waitForServer()) {
        ROS_WARN("Failed to wait for action server");
        return 1;
    }

    ROS_INFO("Wait for GripperCommand action server");
    if (!gripper_client->waitForServer()) {
        ROS_WARN("Failed to wait for action server");
        return 1;
    }

    for (auto& command : commands) {
        if (command->type == Command::Type::Gripper) {
            auto* c = static_cast<GripperCommand*>(command.get());
            control_msgs::GripperCommandGoal goal;
            if (c->open) {
#if 1
                std_msgs::Float64MultiArray msg;
                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].label = "joint";
                msg.layout.dim[0].size = 4;
                msg.layout.dim[0].stride = 1;
                msg.layout.data_offset = 0;
                msg.data = { 60.0, 60.0, 60.0, 137.0 };
                gripper_command_pub->publish(msg);
#else
                continue;
#endif
            } else {
                // goal.command.position = 0.0;

                std_msgs::Float64MultiArray msg;
                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].label = "joint";
                msg.layout.dim[0].size = 4;
                msg.layout.dim[0].stride = 1;
                msg.layout.data_offset = 0;
                msg.data = {200.0, 200.0, 200.0, 137.0};
                gripper_command_pub->publish(msg);
            }

            ROS_INFO("%s gripper", c->open ? "Open" : "Close");
            // auto res = gripper_client.sendGoalAndWait(goal);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            // CheckGripperGrip();
            // if (res.state_ == res.SUCCEEDED) {
            //     ROS_INFO("gripper client returned with state '%s'", res.toString().c_str());
            // } else {
            //     ROS_WARN("gripper client returned with state '%s' (%s)", res.toString().c_str(), res.getText().c_str());
            // }
        } else if (command->type == Command::Type::Trajectory) {
            auto* c = static_cast<TrajectoryCommand*>(command.get());

            control_msgs::FollowJointTrajectoryGoal traj;
#if 0
            traj.trajectory.header.stamp = ros::Time(0);
#else
            // delay execution by a little bit to allow time for the trajectory
            // message to reach the controller
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
                auto positions = std::vector<double>();
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
            auto res = traj_client->sendGoalAndWait(traj);
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

/// Plan and execute (or animate) a sequence of trajectory/gripper commands to
/// manipulate the state of an object.
///
/// \param robot_model The model of the robot
/// \param object_model The model of the object, used for debug visualization
/// \param cspace The collision model of the robot and its environment
/// \param planning_model The planning model of the robot. The reference state
///     is updated to reflect the start state on each query.
/// \param planner The object manipulation planner
/// \param real_start_state The actual current start state, which may be overridden by the planning request
/// \param traj_client Action client for executing joint trajectories
/// \param gripper_client Action client for executing simple gripper clients
/// \param gripper_command_pub Publisher for sending detailed gripper commands
/// \param msg The planning request message
bool ManipulateObject(
    const moveit::core::RobotModelConstPtr& robot_model,
    const smpl::urdf::RobotModel* object_model,
    smpl::collision::CollisionSpace* cspace,
    sbpl_interface::MoveItRobotModel* planning_model,
    ObjectManipPlanner* planner,
    const moveit::core::RobotState* real_start_state,
    FollowJointTrajectoryActionClient* traj_client,
    GripperCommandActionClient* gripper_client,
    ros::Publisher* gripper_command_pub,
    const std::string* stats_path,
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

    auto planning_start = std::chrono::high_resolution_clock::now();

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
        auto planning_finish = std::chrono::high_resolution_clock::now();
        auto planning_duration = std::chrono::duration<double>(planning_finish - planning_start).count();
        if (!stats_path->empty()) {
            auto* f = fopen(stats_path->c_str(), "a");
            if (f != NULL) {
                fprintf(f, "%f,fail\n", planning_duration);
                fclose(f);
            }
        }
        return false;
    }

    ProfilerStop();

    auto planning_finish = std::chrono::high_resolution_clock::now();
    auto planning_duration = std::chrono::duration<double>(planning_finish - planning_start).count();

    /////////////////////////////////////////////
    // Execute (or animate) the resulting plan //
    /////////////////////////////////////////////

    auto execute = !msg->plan_only;

    auto execute_start = std::chrono::high_resolution_clock::now();

    if (execute) {
        ExecuteTrajectory(traj_client, gripper_client, gripper_command_pub, commands);
    } else {
        AnimateTrajectory(robot_model, commands, 5.0);
    }

    auto execute_finish = std::chrono::high_resolution_clock::now();
    auto execution_duration = std::chrono::duration<double>(execute_finish - execute_start).count();

    if (!stats_path->empty()) {
        auto* f = fopen(stats_path->c_str(), "a");
        if (f != NULL) {
            fprintf(f, "%f,%f\n", planning_duration, execution_duration);
            fclose(f);
        }
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

template <class T>
auto interp(const T& src, const T& dst, double t) -> T
{
    return (1.0 - t) * src + t * dst;
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
    auto nh = ros::NodeHandle();
    auto check_state_validity = nh.serviceClient<moveit_msgs::GetStateValidity>(
            "check_state_validity");

    auto interm_state = *move_group->getCurrentState();
    auto manip_traj = robot_trajectory::RobotTrajectory(
            interm_state.getRobotModel(), group_name);
    manip_traj.addSuffixWayPoint(interm_state, 0.0);
    auto ids = (int32_t)0;

    ROS_DEBUG("samples = %d", samples);
    for (auto i = 1; i < samples; ++i) { // skip the first waypoint, assume we have at least two samples
        auto alpha = (double)i / (double)(samples - 1);

        // ROS_INFO("calling sample manifold fn now with alpa  = %f", alpha);
        auto contact_pose = sampler(alpha);

        ROS_DEBUG_STREAM("contact pose is " << contact_pose.matrix());

        auto robot_model = interm_state.getRobotModel();
        auto* group = robot_model->getJointModelGroup(group_name);

        auto consistency_limits = std::vector<double>(group->getVariableCount(), smpl::to_radians(10));

        // TODO: Why don't we require a feasible ik solution at all intermediate
        // waypoints? This could be worse, if we weren't using consistency
        // limits, but it's still pretty bad.

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

            // TODO: interpolate path since for lack of an interface for CCD.

            moveit_msgs::GetStateValidity::Request req;
            moveit_msgs::GetStateValidity::Response res;
            robotStateToRobotStateMsg(interm_state, req.robot_state);
            req.group_name = "right_arm_and_torso";
            if (!check_state_validity.call(req, res)) {
                ROS_WARN("Failed to call check_state_validity service for waypoint %d", i);
                return false;
            }

            if (!res.valid) {
                ROS_WARN("Waypoint %d on manipulation trajectory is invalid", i);
                return false;
            }
        }
        else
            ROS_INFO("unable to call ik ");

    }

    // timestamp waypoint
    auto itp = trajectory_processing::IterativeParabolicTimeParameterization();
    if (!itp.computeTimeStamps(manip_traj)) {
        ROS_ERROR("Failed to compute timestamps");
        return false;
    }

    auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
    robotStateToRobotStateMsg(*move_group->getCurrentState(), plan.start_state_);
    manip_traj.getRobotTrajectoryMsg(plan.trajectory_);

    *oplan = std::move(plan);
    return true;
}

bool OpenGripper(GripperCommandActionClient* gripper_client)
{
    ROS_INFO("Open gripper full function");
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.0841;
    auto res = gripper_client->sendGoalAndWait(gripper_goal);
    return res.state_ == res.SUCCEEDED;
}

bool OpenGripperPartial(ros::Publisher* gripper_command_pub)
{
    std_msgs::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "joint";
    msg.layout.dim[0].size = 4;
    msg.layout.dim[0].stride = 1;
    msg.layout.data_offset = 0;
    msg.data = { 60.0, 60.0, 60.0, 137.0 };
    gripper_command_pub->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    return true;
}

bool CloseGripper(GripperCommandActionClient* gripper_client)
{
    ROS_INFO("Close gripper");
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.0;
    auto res = gripper_client->sendGoalAndWait(gripper_goal);
    return res.state_ == res.SUCCEEDED;
}

// Attempted to implement the crate release logic in such a way that the
// trajectory and gripper commands are generated up-front before execution. This
// way, we could substitute execution on the robot with an animation (that
// also doesn't modify the simulation state). This is possible, but it's going
// to be a bit of a pain since we need to track where the robot should end up
// after each command, to generate the successive command, rather than simply
// querying for the current state of the robot in between each command.
#define DEV_ANIMATE_RELEASE 0

bool ReleaseCrate(
    const moveit::core::RobotState* curr_start_state,
    const smpl::urdf::RobotModel* object_model,
    const std::string& group_name,
    moveit::planning_interface::MoveGroupInterface* move_group,
    FollowJointTrajectoryActionClient* traj_client,
    GripperCommandActionClient* gripper_client,
    ros::Publisher* gripper_command_pub,
    bool plan_only)
{
    // we're not going to perform the crate release in plan-only mode. see note
    // above
    if (plan_only) return true;

    // Rotate the gripper in place
    // Open the gripper
    // While maintaining the same current orientation, move the gripper back by a certain distance
    // Move gripper back to the starting position

    /////////////////////////////////////
    // Initialize Move Group Interface //
    /////////////////////////////////////

    move_group->setPlanningTime(10.0);

    move_group->setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]");
    move_group->setGoalPositionTolerance(0.02);
    move_group->setGoalOrientationTolerance(smpl::to_radians(2.0));

    move_group->setWorkspace(-0.5, -1.5, -0.2, 1.5, 1.5, 1.8);

    auto tool_link_name = "limb_right_tool0";

    if (!gripper_client->waitForServer()) {
        ROS_WARN("Failed to wait for GripperCommand action server");
        return 1;
    }

    ROS_INFO("finished waiting for the GripperCommand action server. out now");

    auto commands = std::vector<std::unique_ptr<Command>>();

    auto start_state = *move_group->getCurrentState();

    //////////////////////
    // Release Manifold //
    //////////////////////

    move_group->setEndEffectorLink(tool_link_name);

    {
        auto release_manifold = [&](double alpha) -> Eigen::Affine3d
        {
            auto s = interp(0.0, -0.15 * M_PI, alpha);

            Eigen::Vector3d rot2(0,0,1);
            double mag = rot2.norm();
            Eigen::AngleAxisd rot(s , rot2/mag);
            auto curr_state = *move_group->getCurrentState();
            auto& tool_transform = curr_state.getGlobalLinkTransform(tool_link_name);
            auto rotate_gripper_pose = tool_transform * rot;

            return rotate_gripper_pose;
        };

        auto samples = 20; //std::max(2, (int)std::round(arc / res));

        auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
        if (!PlanManipulationTrajectory(
                move_group,
                group_name, tool_link_name,
                samples, release_manifold,
                &plan))
        {
            return false;
        }

#if DEV_ANIMATE_RELEASE
//        plan.start_state_; // moveit_msgs::RobotState start_state_;
//        plan.trajectory_; // moveit_msgs::RobotTrajectory
        auto traj = robot_trajectory::RobotTrajectory(move_group->getRobotModel(), group_name);
        traj.setRobotTrajectoryMsg(*curr_start_state, plan.trajectory_);

        auto traj_command = smpl::make_unique<TrajectoryCommand>(std::move(traj));
        commands.push_back(std::move(traj_command));
#else
        auto err = move_group->execute(plan);
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to execute manip trajectory");
            return false;
        }
#endif
    }

#if DEV_ANIMATE_RELEASE
    commands.push_back(smpl::make_unique<GripperCommand>(true));
    if (plan_only) {
        AnimateTrajectory(curr_start_state->getRobotModel(), commands, 5.0);
    } else {
        ExecuteTrajectory(traj_client, gripper_client, gripper_command_pub, commands);
    }
    // TODO: leftoff at this point after initial tests...for this to work we
    // need to know what state the robot should be in after executing the crate
    // opening segments, even in plan-only mode.
    return true;
#endif

    // if (!OpenGripper(gripper_client)) {
    //     ROS_ERROR("Failed to open gripper");
    //     return false;
    // }

    OpenGripperPartial(gripper_command_pub);

    ///////////////////////////
    // move the gripper back //
    ///////////////////////////

    {
        auto curr_state = *move_group->getCurrentState();
        auto& tool_transform = curr_state.getGlobalLinkTransform(tool_link_name);
        auto withdraw_pose =
                tool_transform *
                Eigen::Translation3d(-0.1, 0.0, 0.0);

        move_group->setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]");
        move_group->setGoalPositionTolerance(0.02);
        move_group->setGoalOrientationTolerance(smpl::to_radians(2.0));
        move_group->setPoseTarget(withdraw_pose, tool_link_name);
        auto err = move_group->move();
    }

    //////////////////////////////////
    // move back to the start state //
    //////////////////////////////////

    {
        move_group->setPlannerId("right_arm_and_torso[right_arm_and_torso_ARA_JD_ML]");
        move_group->setGoalJointTolerance(smpl::to_radians(1.0));
        move_group->setJointValueTarget(*curr_start_state);

        auto err = move_group->move();
        if (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return false;
        }
    }

    return true;
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

    // File to store planning statistics in
    auto stats_path = std::string();
    ph.param<std::string>("stats_path", stats_path, "");

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
    JointStateCallback jsfun = [&](const sensor_msgs::JointState::ConstPtr& msg)
    {
        UpdateCurrentState(&state_monitor, *msg);
    };

    auto sub = nh.subscribe("joint_states", 10, jsfun);

    auto traj_client_name = std::string();
    auto gripper_client_name = std::string();
    if (!GetParam(ph, "follow_joint_trajectory_action_name", &traj_client_name) ||
        !GetParam(ph, "gripper_command_action_name", &gripper_client_name))
    {
        return 1;
    }

    ROS_INFO("Create FollowjointTrajectory action client '%s'", traj_client_name.c_str());
    ROS_INFO("Create GripperCommand action client '%s'", gripper_client_name.c_str());

    FollowJointTrajectoryActionClient traj_client(traj_client_name);
    GripperCommandActionClient gripper_client(gripper_client_name);

    auto gripper_command_pub = nh.advertise<std_msgs::Float64MultiArray>(
            "rcta_right_robotiq_controller/command", 10);

    auto move_group_name = std::string("right_arm_and_torso");
    auto move_group =
            moveit::planning_interface::MoveGroupInterface(move_group_name);

    auto autostart = false;

    ManipulateObjectActionServer server(
        "manipulate_object",
        [&](const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
        {
            auto curr_state = GetCurrentState(&state_monitor);
            if (ManipulateObject(
                robot_model,
                &object_model,
                &cspace,
                &planning_model,
                &planner,
                &curr_state,
                &traj_client,
                &gripper_client,
                &gripper_command_pub,
                &stats_path,
                msg))
            {
                if (!ReleaseCrate(
                        &curr_state,
                        &object_model,
                        move_group_name,
                        &move_group,
                        &traj_client,
                        &gripper_client,
                        &gripper_command_pub,
                        msg->plan_only))
                {
                    ROS_ERROR("Failed to release crate");
                    server.setAborted();
                    return;
                }

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
