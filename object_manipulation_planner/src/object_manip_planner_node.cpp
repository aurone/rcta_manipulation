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
#include <moveit_planners_sbpl/planner/moveit_robot_model.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/occupancy_grid.h>

// project includes
#include "object_manip_planner.h"
#include "object_manipulation_model.h"

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

    // TODO: parameterize
    auto group_name = "right_arm_torso_base";
    auto tip_link = "limb_right_tool0";
    auto ik_group_name = "right_arm";

    /////////////////////////////////////////////
    // Load the Robot Model from the URDF/SRDF //
    /////////////////////////////////////////////

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

    // frame of the planning/collision model, for visualization
    std::string planning_frame;
    ph.param<std::string>("planning_frame", planning_frame, "map");

    // Parameters taken from 'world_collision_model' declared in move_group.launch
    auto size_x = 20.0;
    auto size_y = 20.0;
    auto size_z = 2.1;
    auto origin_x = -10.0;
    auto origin_y = -10.0;
    auto origin_z = -0.15;
    auto resolution = 0.05;
    auto max_dist = 0.8;
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

    auto planning_variables = omanip.getPlanningJoints();
    planning_variables.pop_back();
    smpl::collision::CollisionSpace cspace;
    if (!cspace.init(
            &grid,
            *robot_model->getURDF().get(),
            config,
            group_name,
            planning_variables))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    ////////////////////////////
    // Initialize the Planner //
    ////////////////////////////

    ObjectManipPlanner planner;
    if (!Init(&planner, &omanip, &cspace, &grid)) {
        ROS_ERROR("Failed to initialize Object Manipulation Planner");
        return 1;
    }

    // TODO: parameterize
    auto demos = "/home/aurone/data/egraphs/right_arm_torso_base_paths";
    if (!LoadDemonstrations(&planner, demos)) {
        return 1;
    }

    ////////////
    // Inputs //
    ////////////

    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();

    // Update the reference state in the collision model
    for (auto i = 0; i < robot_model->getVariableCount(); ++i) {
        auto& name = robot_model->getVariableNames()[i];
        auto pos = start_state.getVariablePositions()[i];
        if (!cspace.setJointPosition(name, pos)) {
            ROS_ERROR("Failed to set position of joint '%s' to %f in the collision model", name.c_str(), pos);
            return 1;
        }
    }

    // Update the reference state in the planning model
    if (!planning_model.updateReferenceState(start_state)) {
        ROS_ERROR("Failed to update the planning model reference state");
        return 1;
    }

    Eigen::Affine3d object_pose =
            Eigen::Translation3d(1.0, 2.0, 0.4) *
            Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

    auto object_start_state = 0.0;

    auto object_goal_state = 1.0;

    auto allowed_time = 10.0;

    /////////////
    // Outputs //
    /////////////

    robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);

    ///////////
    // Plan! //
    ///////////

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

    {
        moveit_msgs::DisplayTrajectory display;

        display.model_id = robot_model->getName();

        display.trajectory.resize(1);
        trajectory.getRobotTrajectoryMsg(display.trajectory[0]);

        display.trajectory_start;

        display_publisher.publish(display);
    }

    // TODO: convert to FollowJointTrajectoryGoal for execution

    return 0;
}
