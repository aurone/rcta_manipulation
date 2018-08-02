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
    smpl::WorkspaceLatticeEGraph graph;

    smpl::SimpleWorkspaceLatticeActionSpace actions;

    smpl::WorkspaceLattice::Params p;
    if (!graph.init(model, checker, &planner->params, p, &planner->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    if (!InitRomanWorkspaceLatticeActions(&planner->graph, &planner->actions)) {
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
    double object_goal_state,
    double allowed_time,
    robot_trajectory::RobotTrajectory* trajectory)
{
    // TODO: behavior to level out the end effector

    // TODO: goal is to open the object all the way, frame this as a
    // partially-specified joint configuration goal
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
