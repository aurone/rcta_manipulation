#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_planners_sbpl/planner/moveit_robot_model.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice_egraph.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/object_manip_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>

#include "object_manipulation_model.h"

//#include "workspace_lattice_egraph.h"

struct ObjectManipPlanner
{
    smpl::PlanningParams                    params;
    smpl::WorkspaceLatticeEGraph            graph;
    smpl::SimpleWorkspaceLatticeActionSpace actions;
    smpl::ObjectManipulationHeuristic       heuristic;
    smpl::ARAStar                           search;

    ObjectManipPlanner() : search(&graph, &heuristic) { }
};

bool Init(
    ObjectManipPlanner* planner,
    smpl::RobotModel* model,
    smpl::CollisionChecker* checker,
    smpl::OccupancyGrid* grid)
{
    smpl::WorkspaceLatticeEGraph graph;

    smpl::SimpleWorkspaceLatticeActionSpace actions;

    smpl::WorkspaceLattice::Params p;
    if (!graph.init(model, checker, &planner->params, p, &planner->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    if (!InitSimpleWorkspaceLatticeActions(&planner->graph, &planner->actions)) {
        return false;
    }

    if (!planner->heuristic.init(&graph)) {
        ROS_ERROR("Failed to initialize Dijkstra E-Graph Heuristic 3D");
        return false;
    }

    planner->search.allowPartialSolutions(false);
    planner->search.setTargetEpsilon(1.0);
    planner->search.setDeltaEpsilon(1.0);
    planner->search.setImproveSolution(true);
    planner->search.setBoundExpansions(true);
    return true;
}

bool LoadDemonstrations(ObjectManipPlanner* planner, const std::string& path)
{
    if (!planner->graph.loadExperienceGraph(path)) {
        ROS_ERROR("Failed to load experience graph");
        return false;
    }
    return true;
}

bool PlanPath(
    ObjectManipPlanner* planner,
    const moveit::core::RobotState& start_state,
    const Eigen::Affine3d& object_pose,
    double object_start_state,
    double allowed_time,
    double object_goal_state,
    robot_trajectory::RobotTrajectory* trajectory)
{
    // TODO: behavior to level out the end effector

    // goal is to open the object all the way

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;

    smpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = true;
    timing.max_allowed_time_init = std::chrono::seconds(10);
    timing.max_allowed_time = std::chrono::seconds(10);
    std::vector<int> solution;
    int solution_cost;
    bool res = planner->search.replan(timing, &solution, &solution_cost);
    if (!res) {
        ROS_ERROR("Failed to plan path");
        return false;
    }

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
    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);

    auto group_name = "right_arm_torso_base";
    auto tip_link = "limb_right_tool0";
    auto ik_group_name = "right_arm";

    // We basically have to be a ROS node for this to work. We could use
    // RobotModelLoader to load the RobotModel from urdf/srdf strings, but
    // we still wouldn't be able to load kinematics solvers without doing it
    // manually...
    ros::init(argc, argv, "object_manip_planner");
    ros::NodeHandle nh;

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

    ObjectManipulationModel omanip;
    if (!Init(&omanip, &planning_model, 0.0, 1.0)) {
        ROS_ERROR("Failed to initialize Object Manipulation Model");
        return 1;
    }

    //////////////////////////////////////
    // Initialize the Collision Checker //
    //////////////////////////////////////

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

    smpl::collision::CollisionModelConfig config;
    if (!smpl::collision::CollisionModelConfig::Load(nh, config)) {
        ROS_ERROR("Failed to load Collision Model Configuration");
        return 1;
    }

    auto planning_variables = omanip.getPlanningJoints();
    planning_variables.pop_back();
    smpl::collision::CollisionSpace cspace;
    if (!cspace.init(&grid, *robot_model->getURDF().get(), config, group_name, planning_variables)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();

    // TODO: set up the scene (set the initial robot state)
    SV_SHOW_INFO(cspace.getCollisionRobotVisualization());

    ////////////////////////////
    // Initialize the Planner //
    ////////////////////////////

    ObjectManipPlanner planner;
    if (!Init(&planner, &omanip, &cspace, &grid)) {
        ROS_ERROR("Failed to initialize Object Manipulation Planner");
        return 1;
    }
    auto demos = "/home/aurone/data/egraphs/right_arm_torso_base_paths";
    if (!LoadDemonstrations(&planner, demos)) {
        return 1;
    }

    ////////////
    // Inputs //
    ////////////

    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();
    Eigen::Affine3d object_pose =
            Eigen::Translation3d(1.0, 2.0, 0.4) *
            Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
    auto object_start_state = 0.0;
    auto allowed_time = 10.0;

    auto object_goal_state = 1.0;

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
            allowed_time,
            object_goal_state,
            &trajectory))
    {
        ROS_ERROR("Failed to plan path");
        return 1;
    }

    // convert to:
    // (1) moveit_msgs::DisplayTrajectory for debugging
    // (2) FollowJointTrajectoryGoal for execution

    return 0;
}

