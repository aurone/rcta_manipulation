// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <smpl_moveit_interface/planner/moveit_robot_model.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/occupancy_grid.h>

// project includes
#include "assert.h"
#include "object_manip_planner.h"
#include "object_manipulation_model.h"
#include "object_manip_checker.h"

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

    auto display_publisher = ph.advertise<moveit_msgs::DisplayTrajectory>("planned_path", 1);

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
    if (!robot_model) {
        ROS_ERROR("Failed to load Robot Model");
        return 1;
    }

    std::vector<std::string> redundant_joints = { "limb_right_joint3" };
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

    ObjectManipModel omanip;
    if (!Init(&omanip, &planning_model, "hinge", 0.0, 1.0)) {
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
    auto grid = smpl::OccupancyGrid(
            size_x,
            size_y,
            size_z,
            resolution,
            origin_x,
            origin_y,
            origin_z,
            max_dist);

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
            *robot_model->getURDF().get(),
            config,
            group_name,
            omanip.parent_model->getPlanningJoints()))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    ObjectManipChecker checker;
    checker.parent = &cspace;

    ////////////////////////////
    // Initialize the Planner //
    ////////////////////////////

    ROS_INFO("Initialize Object Manipulation Planner");

    ObjectManipPlanner planner;
    if (!Init(&planner, &omanip, &checker, &grid)) {
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

    ////////////
    // Inputs //
    ////////////

    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();

    start_state.setVariablePosition("limb_right_joint2", smpl::to_radians(30.0));
    start_state.setVariablePosition("limb_right_joint4", smpl::to_radians(30.0));
    start_state.setVariablePosition("limb_right_joint6", smpl::to_radians(30.0));
    start_state.setVariablePosition("limb_right_joint7", smpl::to_radians(90.0));
    start_state.setVariablePosition("limb_left_joint1", smpl::to_radians(-90.0));
    start_state.setVariablePosition("limb_left_joint2", smpl::to_radians(90.0));
    start_state.setVariablePosition("limb_left_joint3", smpl::to_radians(90.0));
    start_state.setVariablePosition("limb_left_joint4", smpl::to_radians(180.0));
    start_state.setVariablePosition("limb_left_joint5", smpl::to_radians(-90.0));
    start_state.setVariablePosition("limb_left_joint6", smpl::to_radians(0.0));
    start_state.setVariablePosition("limb_left_joint7", smpl::to_radians(0.0));

//    start_state.setVariablePosition("world_joint/x", 1.0);
//    start_state.setVariablePosition("world_joint/y", 0.4);
//    start_state.setVariablePosition("world_joint/theta", smpl::to_radians(90));

    start_state.setVariablePosition("world_joint/x", 0.0);
    start_state.setVariablePosition("world_joint/y", 0.0);
    start_state.setVariablePosition("world_joint/theta", smpl::to_radians(0));

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

    ROS_INFO("Update start state in collision model");

    for (auto i = 0; i < robot_model->getVariableCount(); ++i) {
        auto& name = robot_model->getVariableNames()[i];
        auto pos = start_state.getVariablePositions()[i];
        if (!cspace.setJointPosition(name, pos)) {
            ROS_ERROR("Failed to set position of joint '%s' to %f in the collision model", name.c_str(), pos);
            return 1;
        }
    }

    ROS_INFO("Update start state in planning model");

    if (!planning_model.updateReferenceState(start_state)) {
        ROS_ERROR("Failed to update the planning model reference state");
        return 1;
    }

    Eigen::Affine3d object_pose =
            Eigen::Translation3d(1.0, 2.0, 0.4) *
            Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

    auto object_start_state = 0.0;
    (void)GetParam(ph, "object_start_position", &object_start_state);

    auto object_goal_state = 1.0;
    (void)GetParam(ph, "object_goal_position", &object_goal_state);

    double allowed_time;
    ph.param("allowed_planning_time", allowed_time, 10.0);

    /////////////
    // Outputs //
    /////////////

    robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);

    ///////////
    // Plan! //
    ///////////

    ROS_INFO("Plan path!");

    if (!PlanPath(
            &planner,
            start_state,
            object_pose,
            object_start_state,
            object_goal_state,
            allowed_time,
            &trajectory))
    {
        ROS_ERROR("Failed to plan path");
        return 1;
    }

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

    //////////////////////////////////////////////////////////////
    // TODO: convert to FollowJointTrajectoryGoal for execution //
    //////////////////////////////////////////////////////////////

    ROS_INFO("Execute trajectory");

    return 0;
}
