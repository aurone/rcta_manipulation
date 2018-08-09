#include "object_manip_planner.h"

// system includes
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <smpl/console/nonstd.h>
#include <smpl/occupancy_grid.h>
#include <smpl/post_processing.h>
#include <smpl/planning_params.h>

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
    planner->checker = checker;

    smpl::WorkspaceLattice::Params p;
    // TODO: parameterize these. one free angle here?
    p.free_angle_res =
    {
        smpl::to_radians(5),        // arm free angle
        smpl::to_radians(5),        // torso joint
        0.05,                       // base x
        0.05,                       // base y
        smpl::to_radians(22.5),     // base theta
        0.00001                     // object z
    };
    p.res_x = 0.05;
    p.res_y = 0.05;
    p.res_z = 0.05;
    p.Y_count = 72;
    p.P_count = 3;
    p.R_count = 72;

    if (!planner->graph.init(model, checker, p, &planner->actions)) {
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

auto MakeGraphStatePrefix(
    const moveit::core::RobotState& state,
    ObjectManipModel* model)
    -> smpl::RobotState
{
    smpl::RobotState s;
    for (auto& joint : model->parent_model->getPlanningJoints()) {
        auto pos = state.getVariablePosition(joint);
        s.push_back(pos);
    }
    return s;
}

void UpdateRobotState(
    moveit::core::RobotState& robot_state,
    const smpl::RobotState& graph_state,
    ObjectManipModel* model)
{
    for (size_t i = 0; i < model->parent_model->getPlanningJoints().size(); ++i) {
        auto varname = model->parent_model->getPlanningJoints()[i];
        robot_state.setVariablePosition(varname, graph_state[i]);
    }
}

bool IsGoal(void* user, const smpl::RobotState& state)
{
    auto* goal = static_cast<smpl::GoalConstraint*>(user);
    auto diff = state.back() - goal->angles.back();
    if (std::fabs(diff) < goal->angle_tolerances.back()) {
        ROS_INFO("Found a goal state (%f vs %f)", state.back(), goal->angles.back());
        return true;
    }

    return false;
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

    ROS_INFO_STREAM("Start state = " << start);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;
    goal.angles.push_back(object_goal_state);
    goal.angle_tolerances.push_back(0.05);
    goal.check_goal = IsGoal;
    goal.check_goal_user = &goal;
    planner->graph.setGoal(goal);

    auto start_id = planner->graph.getStartStateID();
    auto goal_id = planner->graph.getGoalStateID();
    planner->search.set_start(start_id);
    planner->search.set_goal(goal_id);

    ROS_INFO("start state id = %d", start_id);
    ROS_INFO("goal state id = %d", goal_id);

    smpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = true;
    timing.type = smpl::ARAStar::TimeParameters::TIME;
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

    /////////////////
    // smooth path //
    /////////////////

    if (!smpl::InterpolatePath(*planner->checker, path)) {
        ROS_ERROR("Failed to interpolate path");
        return false;
    }

    std::vector<smpl::RobotState> shortcut_path;
    smpl::ShortcutPath(
            planner->model,
            planner->checker,
            path,
            shortcut_path,
            smpl::ShortcutType::JOINT_SPACE);

    //////////////////////////////
    // TODO: Profile Trajectory //
    //////////////////////////////

    //////////////////////////////////////////////////
    // Convert to robot_trajectory::RobotTrajectory //
    //////////////////////////////////////////////////

    for (auto& point : shortcut_path) {
        moveit::core::RobotState state(start_state);
        UpdateRobotState(state, point, planner->model);
        trajectory->addSuffixWayPoint(state, 1.0);
    }

    return true;
}
