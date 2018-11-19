// standard includes
#include <chrono>
#include <string>
#include <thread>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <smpl_moveit_interface/planner/moveit_robot_model.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/stl/memory.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/occupancy_grid.h>
#include <tf/transform_listener.h>
#include <gperftools/profiler.h>

#include <cmu_manipulation_msgs/ManipulateObjectAction.h>

// project includes
#include "assert.h"
#include "object_manip_planner.h"
#include "object_manip_model.h"
#include "object_manip_checker.h"

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

// Little helper for uniform success/error logging
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

bool ExecuteTrajectory(
    const ros::NodeHandle& nh,
    const std::vector<std::unique_ptr<Command>>& commands)
{
    using GripperCommandActionClient =
            actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

    using FollowJointTrajectoryActionClient =
            actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

    std::string traj_client_name;
    std::string gripper_client_name;
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
            traj.trajectory.header.stamp = ros::Time::now();
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

#if ROS_KINETIC
                traj.trajectory.points[i].time_from_start =
                        ros::Duration(c->trajectory.getWayPointDurationFromStart(i));
#else
                traj.trajectory.points[i].time_from_start =
                        ros::Duration(c->trajectory.getWaypointDurationFromStart(i));
#endif

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

bool AnimateTrajectory(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::vector<std::unique_ptr<Command>>& commands)
{
    auto last_wp = moveit::core::RobotState(robot_model);
    for (auto& command : commands) {
        if (command->type == Command::Type::Gripper) {
            if (!ros::ok()) break;

            auto* gripper_cmd = static_cast<GripperCommand*>(command.get());

            std_msgs::ColorRGBA color;
            if (gripper_cmd->open) {
                color.g = 0.5f;
                color.a = 0.9f;
            } else {
                color.r = 0.5f;
                color.a = 0.9f;
            }

            visualization_msgs::MarkerArray ma;
            last_wp.getRobotMarkers(ma, last_wp.getRobotModel()->getLinkModelNames(), color, "trajectory", ros::Duration(0));

            SV_SHOW_INFO(ma);

            ros::Duration(1.0).sleep();
        } else if (command->type == Command::Type::Trajectory) {
            if (!ros::ok()) break;

            auto* trajectory = static_cast<TrajectoryCommand*>(command.get());
            for (auto i = 0; i < trajectory->trajectory.getWayPointCount(); ++i) {
                auto dur = trajectory->trajectory.getWayPointDurations()[i];

                auto& wp = trajectory->trajectory.getWayPoint(i);

                std_msgs::ColorRGBA color;
                color.r = color.g = color.b = 0.5f;
                color.a = 0.9f;

                visualization_msgs::MarkerArray ma;
                wp.getRobotMarkers(ma, wp.getRobotModel()->getLinkModelNames(), color, "trajectory", ros::Duration(0));

                SV_SHOW_INFO(ma);

                ros::Duration(dur).sleep();
//                std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1e3 * dur)));
            }

            last_wp = trajectory->trajectory.getLastWayPoint();
        } else {
            return false;
        }
    }
    return true;
}

using ManipulateObjectActionServer =
        actionlib::SimpleActionServer<cmu_manipulation_msgs::ManipulateObjectAction>;

void ManipulateObject(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const moveit::core::RobotState* real_start_state,
    smpl::collision::CollisionSpace& cspace,
    sbpl_interface::MoveItRobotModel& planning_model,
    ObjectManipPlanner* planner,
    const ros::Publisher& display_publisher,
    const ros::NodeHandle& ph,
    ManipulateObjectActionServer* server,
    const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
{
    moveit::core::RobotState start_state(*real_start_state);

    // TODO: initialize to current state values
//    start_state.setToDefaultValues();

    // because fuck you moveit, plz stahp throwing exceptions...unreferenced
    // but keeping this around in case we explode
    auto robot_has_variable = [&](
        const moveit::core::RobotModel& model,
        const std::string& name)
    {
        auto it = std::find(begin(model.getVariableNames()), end(model.getVariableNames()), name);
        return it != end(model.getVariableNames());
    };

    // hmmm...is this going to barf when there are superflous joints?
    if (!moveit::core::robotStateMsgToRobotState(msg->start_state, start_state)) {
        ROS_ERROR("Failed to update start state");
        server->setAborted();
        return;
    }

    // Visualize the start state
    {
        ros::Duration(1.0).sleep();
        visualization_msgs::MarkerArray ma;
        std_msgs::ColorRGBA color;
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
        ros::Duration(1.0).sleep();
    }

    Eigen::Affine3d object_pose;
    tf::poseMsgToEigen(msg->object_pose, object_pose);

    auto object_start_state = msg->object_start;
    auto object_goal_state = msg->object_goal;

    auto allowed_time = msg->allowed_planning_time;

    /////////////
    // Outputs //
    /////////////

    std::vector<std::unique_ptr<Command>> commands;
    robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);

    ///////////
    // Plan! //
    ///////////

    ROS_INFO("Update start state in collision model");

    for (auto i = 0; i < robot_model->getVariableCount(); ++i) {
        auto& name = robot_model->getVariableNames()[i];
        auto pos = start_state.getVariablePositions()[i];
        if (!cspace.setJointPosition(name, pos)) {
            ROS_ERROR("Failed to set position of joint '%s' to %f in the collision model", name.c_str(), pos);
            server->setAborted();
            return;
        }
    }

    ROS_INFO("Update start state in planning model");

    if (!planning_model.updateReferenceState(start_state)) {
        ROS_ERROR("Failed to update the planning model reference state");
        server->setAborted();
        return;
    }

    ROS_INFO("Plan path!");

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
        server->setAborted();
        return;
    }
    ProfilerStop();

    MakeRobotTrajectory(&commands, &trajectory);

    //////////////////////////////
    // display the planned path //
    //////////////////////////////

    ROS_INFO("Display trajectory");

    {
        moveit_msgs::DisplayTrajectory display;

        display.model_id = robot_model->getName();

        display.trajectory.resize(1);
        trajectory.getRobotTrajectoryMsg(display.trajectory[0]);

        moveit::core::robotStateToRobotStateMsg(
                trajectory.getFirstWayPoint(),
                display.trajectory_start);

        display_publisher.publish(display);
    }

    auto execute = !msg->plan_only;

    if (execute) {
        ExecuteTrajectory(ph, commands);
    } else {
        AnimateTrajectory(robot_model, commands);
    }

    server->setSucceeded();
}

// 1. Generate a trajectory using the model of the object
// 2. Save the example trajectory as a demonstration
// 3. Construct state space: (base/x, base/y, base/theta, torso, ee/x, ee/y,
//    ee/z, ee/yaw, arm/free_angle)
// 4. Construct a Workspace Lattice with a special action set.
//  * The original action space, replicated for all values of z
//  *
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_manip_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    auto display_publisher =
            ph.advertise<moveit_msgs::DisplayTrajectory>("planned_path", 1);

    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);

    ros::Duration(0.5).sleep(); // let publisher set up

    std::string group_name;
    std::string tip_link;
    std::string ik_group_name;
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

    robot_model_loader::RobotModelLoader loader;
    auto robot_model = loader.getModel();
    if (robot_model == NULL) {
        ROS_ERROR("Failed to load Robot Model");
        return 1;
    }

    std::vector<std::string> redundant_joints;
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

    if (!planning_model.setPlanningLink(tip_link)) {
        ROS_ERROR("Failed to set planning link");
        return 1;
    }

    auto object_min_position = 0.0;
    auto object_max_position = 1.0;
    ObjectManipModel omanip;
    if (!Init(
            &omanip,
            &planning_model,
            "hinge",
            object_min_position,
            object_max_position))
    {
        ROS_ERROR("Failed to initialize Object Manipulation Model");
        return 1;
    }

    //////////////////////////////////////
    // Initialize the Collision Checker //
    //////////////////////////////////////

    ROS_INFO("Initialize Collision Checker");

    // frame of the planning/collision model, for visualization
    std::string planning_frame;
    ph.param<std::string>("planning_frame", planning_frame, "map");

    double size_x, size_y, size_z, origin_x, origin_y, origin_z, resolution, max_dist;
    ros::NodeHandle gh(ph, "grid");
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

    smpl::collision::CollisionModelConfig config;
    if (!smpl::collision::CollisionModelConfig::Load(nh, config)) {
        ROS_ERROR("Failed to load Collision Model Configuration");
        return 1;
    }

    // NOTE: Initialize CollisionSpace with the planning joints of the parent
    // (non-object) model, since the CollisionSpace has no knowledge of the
    // object joint. Also, it's pure luck that the CollisionSpace ignores "too
    // large" vectors.
    smpl::collision::CollisionSpace cspace;
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
    ochecker.parent = &cspace;

    ////////////////////////////
    // Initialize the Planner //
    ////////////////////////////

    ROS_INFO("Initialize Object Manipulation Planner");

    ObjectManipPlanner planner;
    if (!Init(&planner, &omanip, &ochecker, &grid)) {
        ROS_ERROR("Failed to initialize Object Manipulation Planner");
        return 1;
    }

    std::string demos;
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

    auto listener = boost::make_shared<tf::TransformListener>();
    auto state_monitor =
            smpl::make_unique<planning_scene_monitor::CurrentStateMonitor>(
                    robot_model, listener);
    state_monitor->startStateMonitor();

    auto autostart = false;
    ManipulateObjectActionServer server(
            "manipulate_object",
            [&](const cmu_manipulation_msgs::ManipulateObjectGoal::ConstPtr& msg)
            {
                auto curr_state = state_monitor->getCurrentState();
                return ManipulateObject(
                        robot_model,
                        group_name,
                        curr_state.get(),
                        cspace,
                        planning_model,
                        &planner,
                        display_publisher,
                        ph,
                        &server,
                        msg);
            },
            autostart);

    ROS_INFO("Start action server");
    server.start();
    ros::spin();

    return 0;
}

