#include "object_manip_planner.h"

// system includes
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <smpl/heuristic/object_manip_heuristic.h>
#include <smpl/occupancy_grid.h>

// project includes
#include "object_manipulation_model.h"

ObjectManipPlanner::ObjectManipPlanner() : search(&graph, &heuristic) { }

bool Init(
    ObjectManipPlanner* planner,
    ObjectManipModel* model,
    smpl::CollisionChecker* checker,
    smpl::OccupancyGrid* grid)
{
    planner->model = model;

    smpl::WorkspaceLattice::Params p;
    if (!planner->graph.init(model, checker, &planner->params, p, &planner->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    if (!InitRomanWorkspaceLatticeActions(&planner->graph, &planner->actions)) {
        return false;
    }

    if (!planner->heuristic.init(&planner->graph)) {
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
    // TODO: store demonstrations in the local frame of the object and transform
    // demonstration prior to each planning query
    if (!planner->graph.loadExperienceGraph(path)) {
        ROS_ERROR("Failed to load experience graph");
        return false;
    }
    return true;
}

auto MakeGraphStatePrefix(const moveit::core::RobotState& state, ObjectManipModel* model)
    -> smpl::RobotState
{
    return { }; // TODO: extract the joint variables from the state that correspond to planning variables
}

bool PlanPath(
    ObjectManipPlanner* planner,
    const moveit::core::RobotState& start_state,
    const Eigen::Affine3d& object_pose,
    double object_start_state,
    double object_goal_state,
    double allowed_time,
    robot_trajectory::RobotTrajectory* trajectory)
{
    // TODO: behavior to level out the end effector

    auto start = MakeGraphStatePrefix(start_state, planner->model);
    start.push_back(object_start_state);
    planner->graph.setStart(start);

    // TODO: goal is to open the object all the way, frame this as a
    // partially-specified joint configuration goal
    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    planner->graph.setGoal(goal);

    smpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = true;
    timing.max_allowed_time_init = smpl::to_duration(allowed_time);
    timing.max_allowed_time = smpl::to_duration(allowed_time);
    std::vector<int> solution;
    int solution_cost;
    bool res = planner->search.replan(timing, &solution, &solution_cost);
    if (!res) {
        ROS_ERROR("Failed to plan path");
        return false;
    }

    std::vector<smpl::RobotState> path;
    if (!planner->graph.extractPath(solution, path)) {
        ROS_ERROR("Failed to extract path");
        return false;
    }

    // TODO: smooth path

    // TODO: profile trajectory

    // TODO: convert to robot trajectory by extracting planning variables that
    // correspond to robot state variables

    return true;
}
