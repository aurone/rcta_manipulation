#include "object_manip_planner.h"

// system includes
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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
    // TODO: configurate these
    p.free_angle_res =
    {
        smpl::to_radians(5),        // arm free angle
        smpl::to_radians(5),        // torso joint
        smpl::to_radians(22.5),     // base theta
        0.05,                       // base x
        0.05,                       // base y
        0.00001                     // object z
    };
    p.res_x = 0.05;
    p.res_y = 0.05;
    p.res_z = 0.05;
    p.Y_count = 36; // resolution = 10 degrees
    p.P_count = 19; // resolution = 10 degrees
    p.R_count = 36; // resolution = 10 degrees

    if (!planner->graph.init(model, checker, p, &planner->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    if (!InitRomanWorkspaceLatticeActions(&planner->graph, &planner->actions)) {
        return false;
    }

    if (!Init(&planner->heuristic, &planner->graph)) {
        ROS_ERROR("Failed to initialize Object Manip Heuristic");
        return false;
    }

    // define parameters here for now, TODO: configurate
    ObjectManipPlannerParams params;
    params.use_rotation = true; //false;
    params.disc_rotation_heuristic = true;
    params.rot_db = smpl::to_radians(2);
    params.heading_condition = ObjectManipPlannerParams::HeadingCondition::Discrete;
    params.heading_thresh = 0.05; // 0.35, 0.1, 0.15

    params.disc_position_heuristic = true;
    params.pos_db = 0.1;

    params.rot_weight = 0.45 / smpl::to_radians(45);

    params.base_weight = 10.0;
    params.combination = ObjectManipPlannerParams::CombinationMethod::Max;

    // any necessary conversions should go here
    planner->heuristic.heading_thresh = params.heading_thresh;
    planner->heuristic.theta_db = params.rot_db;
    planner->heuristic.pos_db = params.pos_db;
    planner->heuristic.theta_normalizer = params.rot_weight;
    planner->heuristic.h_base_weight = params.base_weight;
    planner->heuristic.use_rotation = params.use_rotation;
    planner->heuristic.heading_condition = (int)params.heading_condition;
    planner->heuristic.disc_rotation_heuristic = params.disc_rotation_heuristic;
    planner->heuristic.disc_position_heuristic = params.disc_position_heuristic;

#if 0
    planner->search.allowPartialSolutions(false);
    planner->search.setTargetEpsilon(1.0);
    planner->search.setDeltaEpsilon(1.0);
    planner->search.setImproveSolution(true);
    planner->search.setBoundExpansions(true);
#else
    planner->search.set_initialsolution_eps(100.0);
#endif
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

    planner->heuristic.updateStart(start);

    ROS_INFO_STREAM("Start state = " << start);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;
    goal.angles.push_back(object_goal_state);
    goal.angle_tolerances.push_back(0.05);
    goal.check_goal = IsGoal;
    goal.check_goal_user = &goal;
    planner->graph.setGoal(goal);

    planner->heuristic.updateGoal(goal);

    auto start_id = planner->graph.getStartStateID();
    auto goal_id = planner->graph.getGoalStateID();
    planner->search.set_start(start_id);
    planner->search.set_goal(goal_id);

    ROS_INFO("start state id = %d", start_id);
    ROS_INFO("goal state id = %d", goal_id);

    std::vector<int> solution;
    int solution_cost;

#if 0
    smpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = true;
    timing.type = smpl::ARAStar::TimeParameters::TIME;
    timing.max_allowed_time_init = smpl::to_duration(allowed_time);
    timing.max_allowed_time = smpl::to_duration(allowed_time);
    bool res = planner->search.replan(timing, &solution, &solution_cost);
#else
    bool res = planner->search.replan(allowed_time, &solution, &solution_cost);
#endif

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

#if 0
    std::vector<smpl::RobotState> shortcut_path;
    smpl::ShortcutPath(
            planner->model,
            planner->checker,
            path,
            shortcut_path,
            smpl::ShortcutType::JOINT_SPACE);
#else
    auto shortcut_path = path;
#endif

    //////////////////////////////////////////////////
    // Convert to robot_trajectory::RobotTrajectory //
    //////////////////////////////////////////////////

    for (auto& point : shortcut_path) {
        moveit::core::RobotState state(start_state);
        UpdateRobotState(state, point, planner->model);
        trajectory->addSuffixWayPoint(state, 1.0);
    }

    ////////////////////////
    // Profile Trajectory //
    ////////////////////////

    trajectory_processing::IterativeParabolicTimeParameterization profiler;
    profiler.computeTimeStamps(*trajectory);

    return true;
}
