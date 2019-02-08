#include "object_manip_planner.h"

// system includes
#include <boost/filesystem.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <smpl/console/nonstd.h>
#include <smpl/occupancy_grid.h>
#include <smpl/post_processing.h>
#include <smpl/planning_params.h>
#include <smpl/stl/memory.h>

// project includes
#include "object_manip_model.h"
#include "object_manip_heuristic.h"
#include "variables.h"

ObjectManipPlanner::ObjectManipPlanner() : search(&graph, &heuristic) { }

bool Init(
    ObjectManipPlanner* planner,
    ObjectManipModel* model,
    smpl::CollisionChecker* checker,
    smpl::OccupancyGrid* grid,
    const ObjectManipPlannerParams* params)
{
    planner->model = model;
    planner->checker = checker;

    smpl::WorkspaceLattice::Params p;
    // TODO: configurate these
    p.free_angle_res =
    {
        smpl::to_radians(5),        // arm free angle
        0.05,                       // base z
        smpl::to_radians(5),        // torso joint
        smpl::to_radians(22.5),     // base theta
        0.05,                       // base x
        0.05,                       // base y
        1.0                         // object z
    };

    p.res_x = 0.025;
    p.res_y = 0.025;
    p.res_z = 0.025;
    p.Y_count = 36; // resolution = 10 degrees
    p.P_count = 19; // resolution = 10 degrees
    p.R_count = 36; // resolution = 10 degrees

    if (!planner->graph.init(model, checker, p, &planner->actions)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    planner->graph.m_heuristic = &planner->heuristic;

    if (!InitRomanWorkspaceLatticeActions(&planner->graph, &planner->actions)) {
        ROS_ERROR("Failed to initialize Roman Workspace Lattice Action Space");
        return false;
    }

    if (!Init(&planner->heuristic, &planner->graph)) {
        ROS_ERROR("Failed to initialize Object Manip Heuristic");
        return false;
    }

    // any necessary conversions should go here
    planner->heuristic.heading_thresh = params->heading_thresh;
    planner->heuristic.theta_db = params->rot_db;
    planner->heuristic.pos_db = params->pos_db;
    planner->heuristic.theta_normalizer = params->rot_weight;
    planner->heuristic.h_base_weight = params->base_weight;
    planner->heuristic.use_rotation = params->use_rotation;
    planner->heuristic.heading_condition = (int)params->heading_condition;
    planner->heuristic.disc_rotation_heuristic = params->disc_rotation_heuristic;
    planner->heuristic.disc_position_heuristic = params->disc_position_heuristic;

    planner->search.allowPartialSolutions(false);
    planner->search.setTargetEpsilon(1.0);
    planner->search.setDeltaEpsilon(1.0);
    planner->search.setImproveSolution(true);
    planner->search.setBoundExpansions(true);

    planner->search.set_initialsolution_eps(params->w_heuristic);
    return true;
}

bool LoadDemonstrations(ObjectManipPlanner* planner, const std::string& path)
{
    auto p = boost::filesystem::path(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        auto demo_path = std::vector<smpl::RobotState>();
        // TODO: maybe this function should rearrange the variables in the
        // waypoint if they are not identical...oh well...we'll modify
        // the data for now
        if (!smpl::ParseExperienceGraphFile(filepath, planner->model, demo_path)) {
            continue;
        }

        // TODO: We may want to parse the demonstration file ourselves here, so
        // the header information is available to match up the joint variables
        // with their order in the planning model. We get lucky in most cases
        // because the order of the variables recorded in the demonstration
        // happens to match the order in the planning model.

        planner->demos.push_back(std::move(demo_path));
    }

    return true;
}

auto MakeGraphStatePrefix(
    const moveit::core::RobotState& state,
    ObjectManipModel* model)
    -> smpl::RobotState
{
    auto s = smpl::RobotState();
    for (auto& var : model->parent_model->getPlanningJoints()) {
        auto pos = state.getVariablePosition(var);
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

struct PlannerAndGoal
{
    RomanObjectManipLattice* graph;
    smpl::GoalConstraint* goal;
};

bool IsGoal(void* user, const smpl::RobotState& state)
{
    auto* data = static_cast<PlannerAndGoal*>(user);
    auto* graph = data->graph;
    auto* goal = data->goal;

    auto z_index = (int)state[HINGE];
    auto goal_z_index = (int)goal->angles.back();

    if (z_index == goal_z_index) {
        ROS_INFO("Found a goal state (%d vs %d)", z_index, goal_z_index);
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
    std::vector<std::unique_ptr<Command>>* commands)
{
    // the demonstration (the pose of the robot) is stored in the frame of the
    // object -> transform the demonstration into the global frame
    planner->graph.clearExperienceGraph();
    planner->graph.m_goal_entry = NULL;
    planner->graph.m_goal_state_id = -1;
    planner->graph.m_start_entry = NULL;
    planner->graph.m_start_state_id = -1;
    for (auto* state : planner->graph.m_states) {
        delete state;
    }
    planner->graph.m_states.clear();
    planner->graph.m_state_to_id.clear();

    smpl::WorkspaceCoord fake_coord;
    planner->graph.m_goal_state_id = planner->graph.createState(fake_coord);
    planner->graph.m_goal_entry = planner->graph.getState(planner->graph.m_goal_state_id);

    // TODO: assuming here that we have a single demonstration, this is going to
    // get hairy...Also, changing the RobotModel is considered very bad by smpl
    // conventions. In this case we get lucky. The maximum object index, which
    // changes per demonstration, is used for two things:
    // (1) adjusting the discretization so that maximum and minimum bounds are
    // reachable for bounded state variables. Our discretization of 1 should
    // never need to change here
    // (2) Checking joint limits. This would invalidate/validate some states,
    // but we're clearing the state of the search between each call.
    planner->model->min_object_pos = (double)0;
    planner->model->max_object_pos = (double)planner->demos.front().size() - 1;

    for (auto& demo : planner->demos) {
        auto transformed_demo = demo;
        for (auto& point : transformed_demo) {
            auto T_obj_robot = Eigen::Affine3d(
                    Eigen::Translation3d(
                            point[WORLD_JOINT_X],
                            point[WORLD_JOINT_Y],
                            point[WORLD_JOINT_Z]) *
                    Eigen::AngleAxisd(
                            point[WORLD_JOINT_THETA],
                            Eigen::Vector3d::UnitZ()));

            // TODO: this is obnoxiously sensitive. Transforming the
            // demonstration to object frame during recording and back to world
            // frame, even if the relative pose is about the same, causes the
            // pose of the robot to be slightly off due to precision. This is
            // causing some of the nominal examples to not work anymore. The
            // rounding here is a hack to avoid that for now, but the planner
            // should be made robust to these situations.
            auto T_world_robot = Eigen::Affine3d(object_pose * T_obj_robot);
            point[WORLD_JOINT_X] = round(1000.0 * T_world_robot.translation().x()) / 1000.0;
            point[WORLD_JOINT_Y] = round(1000.0 * T_world_robot.translation().y()) / 1000.0;
            point[WORLD_JOINT_THETA] = round(1000.0 * smpl::get_nearest_planar_rotation(Eigen::Quaterniond(T_world_robot.rotation()))) / 1000.0;
            point[WORLD_JOINT_Z] = round(1000.0 * T_world_robot.translation().z()) / 1000.0;
        }

        planner->graph.insertExperienceGraphPath(transformed_demo);
    }

    auto start = MakeGraphStatePrefix(start_state, planner->model);

    // TODO: maybe find the closest z-value?
    auto closest_z_index = [&](double z)
    {
        auto closest_z_index = -1;
        auto closest_z_dist = std::numeric_limits<double>::infinity();
        for (auto i = 0; i < planner->graph.m_demo_z_values.size(); ++i) {
            auto z_value = planner->graph.m_demo_z_values[i];
            auto dist = std::fabs(z - z_value);
            if (dist < closest_z_dist) {
                closest_z_dist = dist;
                closest_z_index = i;
            }
        }
        return closest_z_index;
    };

    auto start_z_index = closest_z_index(object_start_state);

    start.push_back((double)start_z_index);
    if (!planner->graph.setStart(start)) {
        ROS_ERROR("Failed to set start");
        return false;
    }

    planner->heuristic.updateStart(start);

    ROS_INFO_STREAM("Start state = " << start);

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;

    // Hijacking the goal state/tolerance vectors to store the goal position
    // of the object
    goal.angles.push_back(closest_z_index(object_goal_state));
    goal.angle_tolerances.push_back(0.05);
    goal.check_goal = IsGoal;

    PlannerAndGoal goal_data;
    goal_data.goal = &goal;
    goal_data.graph = &planner->graph;
    goal.check_goal_user = &goal_data;

    planner->graph.setGoal(goal);

    planner->heuristic.updateGoal(goal);

    ClearActionCache(&planner->graph);
    planner->search.force_planning_from_scratch_and_free_memory();

    auto start_id = planner->graph.getStartStateID();
    auto goal_id = planner->graph.getGoalStateID();
    planner->search.set_start(start_id);
    planner->search.set_goal(goal_id);

    ROS_INFO("start state id = %d", start_id);
    ROS_INFO("goal state id = %d", goal_id);

    auto solution = std::vector<int>();
    auto solution_cost = 0;

#if 1
    smpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = false;
    timing.type = smpl::ARAStar::TimeParameters::TIME;
    timing.max_allowed_time_init = smpl::to_duration(allowed_time);
    timing.max_allowed_time = smpl::to_duration(allowed_time);
    bool res = planner->search.replan(timing, &solution, &solution_cost);
#else
    auto plan_start = std::chrono::high_resolution_clock::now();
    bool res = planner->search.replan(allowed_time, &solution, &solution_cost);
    auto plan_finish = std::chrono::high_resolution_clock::now();
    ROS_INFO("Planning finished after %f seconds", smpl::to_seconds(plan_finish - plan_start));
#endif

    if (!res) {
        ROS_ERROR("Failed to plan path after %d expansions (%f seconds)", planner->search.get_n_expands(), planner->search.get_final_eps_planning_time());
        return false;
    }

    ROS_INFO("Found path through %zu states with cost %d in %d expansions (%f seconds)", solution.size(), solution_cost, planner->search.get_n_expands(), planner->search.get_final_eps_planning_time());

    using RobotPath = std::vector<smpl::RobotState>;
    RobotPath path;
    if (!planner->graph.extractPath(solution, path)) {
        ROS_ERROR("Failed to extract path");
        return false;
    }

    // partition the path into several path segments based on the action
    // type
    auto segments = std::vector<RobotPath>();
    auto segment_types = std::vector<TransitionType::Type>();
    auto segment_type = TransitionType::Type(path.front().back());
    auto segment = RobotPath();
    for (auto i = 0; i < path.size(); ++i) {
        auto& point = path[i];
        auto type = TransitionType::Type(point.back());
        if (type != segment_type) {
            // record this segment
            segments.push_back(segment);
            segment_types.push_back(segment_type);

            // begin a new segment
            segment.clear();

            segment.push_back(segments.back().back());
            // add the final point of the previous segment as the first waypoint
            // on this segment

            segment_type = type;
        }
        segment.push_back(point);
        segment.back().pop_back(); // remove the type information
    }
    segments.push_back(segment);
    segment_types.push_back(segment_type);

    // Action-specific things to do:
    // * Before a grasp action, open the gripper
    // * After a grasp action, close the gripper
    // * Don't apply shortcutting whenever the state of the object is being
    //   modified, i.e. any edges that were introduced by the demonstration,
    //   includes adjacent e-graph edges and shortcut edges, and z-edges
    //   unless you have some fancy way of connecting things so that they
    //   remain on the constraint manifold

    //////////////////////////
    // smooth path segments //
    //////////////////////////

    for (auto i = 0; i < segments.size(); ++i) {
        auto& segment = segments[i];
        auto type = segment_types[i];

        ROS_INFO("Smooth path of type %s", to_cstring(type));

        if (!smpl::InterpolatePath(*planner->checker, segment)) {
            ROS_ERROR("Failed to interpolate path");
            return false;
        }

        switch (type) {
        case TransitionType::OrigStateOrigSucc:     // yes
        case TransitionType::OrigStateBridgeSucc:   // yes
        case TransitionType::EGraphStateBridgeSucc: // yes
        case TransitionType::PreGraspAmpSucc:       // yes
        case TransitionType::SnapSucc:              // yes
        {
            ROS_INFO("Shortcut path with %zu waypoints", segment.size());
            auto shortcut = std::vector<smpl::RobotState>();
            auto type = smpl::ShortcutType::JOINT_POSITION_VELOCITY_SPACE;
            smpl::ShortcutPath(planner->model, planner->checker, segment, shortcut, type);
            (void)smpl::InterpolatePath(*planner->checker, shortcut);
            segment = std::move(shortcut);
            break;
        }
        case TransitionType::OrigStateZSucc:        // no
        case TransitionType::EGraphStateAdjSucc:    // no
        case TransitionType::EGraphStateZSucc:      // no
        case TransitionType::GraspSucc:             // no
        case TransitionType::PreGraspSucc:          // no
        case TransitionType::ShortcutSucc:          // no
        {
            if (!smpl::InterpolatePath(*planner->checker, segment)) {
                ROS_ERROR("Failed to interpolate path");
                return false;
            }
            break;
        }
        }
    }

    //////////////////////////////////////////////////////////////////////
    // Convert to a sequence of interleaved trajectory/gripper commands //
    //////////////////////////////////////////////////////////////////////

    auto MakeRobotTrajectory = [&](const std::vector<smpl::RobotState>& path)
    {
        robot_trajectory::RobotTrajectory traj(
                start_state.getRobotModel(),
                "right_arm_torso_base");
        for (auto& point : path) {
            moveit::core::RobotState state(start_state);
            UpdateRobotState(state, point, planner->model);
            traj.addSuffixWayPoint(state, 1.0);
        }

        trajectory_processing::IterativeParabolicTimeParameterization profiler;
        profiler.computeTimeStamps(traj);
        return traj;
    };

    // create robot trajectory for the first segment
    auto& first_segment = segments.front();
    commands->push_back(smpl::make_unique<TrajectoryCommand>(MakeRobotTrajectory(first_segment)));

    for (auto i = 1; i < segments.size(); ++i) {
        if (segment_types[i] == TransitionType::GraspSucc) {
            commands->push_back(smpl::make_unique<GripperCommand>(true));
        }

        auto cmd = smpl::make_unique<TrajectoryCommand>(
                MakeRobotTrajectory(segments[i]));
        commands->push_back(std::move(cmd));

        if (segment_types[i] == TransitionType::GraspSucc) {
            commands->push_back(smpl::make_unique<GripperCommand>(false));
        }
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
    std::vector<std::unique_ptr<Command>> commands;
    if (!PlanPath(
            planner,
            start_state,
            object_pose,
            object_start_state,
            object_goal_state,
            allowed_time,
            &commands))
    {
        return false;
    }

    MakeRobotTrajectory(&commands, trajectory);
    return true;
}

void MakeRobotTrajectory(
    const std::vector<std::unique_ptr<Command>>* commands,
    robot_trajectory::RobotTrajectory* traj)
{
    // remove all gripper commands and squash all trajectories
    for (auto& command : (*commands)) {
        if (command->type == Command::Type::Trajectory) {
            auto* c = static_cast<TrajectoryCommand*>(command.get());
            for (auto i = 0; i < c->trajectory.getWayPointCount(); ++i) {
                traj->addSuffixWayPoint(c->trajectory.getWayPoint(i), 1.0);
            }
        }
    }

    trajectory_processing::IterativeParabolicTimeParameterization profiler;
    profiler.computeTimeStamps(*traj);
}

