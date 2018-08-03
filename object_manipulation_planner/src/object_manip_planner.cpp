#include "object_manip_planner.h"

// system includes
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <smpl/occupancy_grid.h>

// project includes
#include "object_manipulation_model.h"
#include "object_manip_heuristic.h"

ObjectManipPlanner::ObjectManipPlanner() : search(&graph, &heuristic) { }

bool Init(
    ObjectManipPlanner* planner,
    ObjectManipModel* model,
    smpl::CollisionChecker* checker,
    smpl::OccupancyGrid* grid)
{
    planner->model = model;

    smpl::WorkspaceLattice::Params p;
    // TODO: parameterize these. one free angle here?
    p.free_angle_res = { smpl::angles::to_radians(5) };
    p.res_x = 0.05;
    p.res_y = 0.05;
    p.res_z = 0.05;
    p.Y_count = 72;
    p.P_count = 3;
    p.R_count = 72;

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

#if 1
    // when the base distance is above this threshold, the base theta
    // term in the heuristic includes turning toward the nearest e-graph
    // state, and then turning to the orientation of the e-graph state;
    // otherwise, the base theta term only includes turning toward the
    // nearest e-graph state
    auto heading_thresh = 0.05; // cabinet1, cabinet2@35
    //heading_thresh: 0.1
    //heading_thresh: 0.15 // cabinet_demo@60

    // tolerance before the base theta term is dropped to 0
    auto theta_db = 0.0349066;

    // tolerance before the base position term is dropped to 0
    auto pos_db = 0.1; // cabinet1, cabinet2@35
    //pos_db: 0.2 // cabinet_demo@60
#endif

    planner->heuristic.heading_thresh = 0.05;
    planner->heuristic.theta_db = theta_db;
    planner->heuristic.pos_db = pos_db;

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
    smpl::RobotState s;
    for (auto& joint : model->parent_model->getPlanningJoints()) {
        auto pos = state.getVariablePosition(joint);
        s.push_back(pos);
    }
    return s;
}

bool IsGoal(void* user, const smpl::RobotState& state)
{
    return std::fabs(state.back() - static_cast<smpl::GoalConstraint*>(user)->angles.back()) < 0.1;
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
    if (!planner->graph.setStart(start)) {
        ROS_ERROR("Failed to set start");
        return false;
    }

    // TODO: goal is to open the object all the way, frame this as a
    // partially-specified joint configuration goal
    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;
    goal.angles.push_back(object_goal_state);
    goal.check_goal = IsGoal;
    goal.check_goal_user = &goal;
    planner->graph.setGoal(goal);

    auto start_id = planner->graph.getStartStateID();
    auto goal_id = planner->graph.getGoalStateID();
    planner->search.set_start(start_id);
    planner->search.set_goal(goal_id);

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

    ROS_INFO("Found path through %zu states", solution.size());

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
