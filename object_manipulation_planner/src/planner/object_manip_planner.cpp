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
#include <smpl/spatial.h>
#include <smpl/angles.h>

// project includes
#include "object_manip_model.h"
#include "object_manip_heuristic.h"
#include "object_manip_checker.h"
#include "variables.h"

// TODO: if you change me, change me in object_manip_heuristic
#define USE_UNIQUE_GOAL 0

ObjectManipPlanner::ObjectManipPlanner() : search(&graph, &heuristic) { }

bool Init(
    ObjectManipPlanner* planner,
    ObjectManipModel* model,
    ObjectManipChecker* checker,
    smpl::OccupancyGrid* grid,
    const ObjectManipPlannerParams* params)
{
    planner->model = model;
    planner->checker = checker;

    // TODO: configurate all of this
    auto wsp = smpl::WorkspaceLattice::Params{ };

    wsp.res_x = 0.025;
    wsp.res_y = 0.025;
    wsp.res_z = 0.025;
    wsp.Y_count = 36; // resolution = 10 degrees
    wsp.P_count = 19; // resolution = 10 degrees
    wsp.R_count = 36; // resolution = 10 degrees

    wsp.free_angle_res =
    {
        0.05,                       // base x
        0.05,                       // base y
        0.05,                       // base z
        smpl::to_radians(22.5),     // base yaw
        smpl::to_radians(22.5),     // base pitch
        smpl::to_radians(22.5),     // base roll
        smpl::to_radians(5),        // torso joint
        smpl::to_radians(5),        // arm free angle
        1.0                         // object z
    };

    if (!planner->graph.init(model, checker, wsp, &planner->actions)) {
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

        // TODO: We may want to parse the demonstration file ourselves here, so
        // the header information is available to match up the joint variables
        // with their order in the planning model. We get lucky in most cases
        // because the order of the variables recorded in the demonstration
        // happens to match the order in the planning model.
        ROS_INFO("Load demonstration from '%s'", filepath.c_str());
        if (!smpl::ParseExperienceGraphFile(filepath, planner->model, demo_path)) {
            continue;
        }

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

auto MakePlannerState(
    const moveit::core::RobotState& state,
    ObjectManipModel* model,
    double z_index)
    -> smpl::RobotState
{
    auto s = smpl::RobotState(VARIABLE_COUNT);
    s[WORLD_JOINT_X] = state.getVariablePosition("world_joint/x");
    s[WORLD_JOINT_Y] = state.getVariablePosition("world_joint/y");
    s[WORLD_JOINT_Z] = 0.0;
    s[WORLD_JOINT_YAW] = state.getVariablePosition("world_joint/theta");
    s[WORLD_JOINT_PITCH] = 0.0;
    s[WORLD_JOINT_ROLL] = 0.0;
    s[TORSO_JOINT1] = state.getVariablePosition("torso_joint1");
    s[LIMB_JOINT1] = state.getVariablePosition("limb_right_joint1");
    s[LIMB_JOINT2] = state.getVariablePosition("limb_right_joint2");
    s[LIMB_JOINT3] = state.getVariablePosition("limb_right_joint3");
    s[LIMB_JOINT4] = state.getVariablePosition("limb_right_joint4");
    s[LIMB_JOINT5] = state.getVariablePosition("limb_right_joint5");
    s[LIMB_JOINT6] = state.getVariablePosition("limb_right_joint6");
    s[LIMB_JOINT7] = state.getVariablePosition("limb_right_joint7");
    s[HINGE] = z_index;
    return s;
}

void UpdateRobotState(
    moveit::core::RobotState& robot_state,
    const smpl::RobotState& graph_state,
    ObjectManipModel* model)
{
#if 1
    robot_state.setVariablePosition("world_joint/x", graph_state[WORLD_JOINT_X]);
    robot_state.setVariablePosition("world_joint/y", graph_state[WORLD_JOINT_Y]);
    // robot_state.setVariablePosition("", graph_state[WORLD_JOINT_Z]);
    robot_state.setVariablePosition("world_joint/theta", graph_state[WORLD_JOINT_YAW]);
    // robot_state.setVariablePosition("", graph_state[WORLD_JOINT_PITCH]);
    // robot_state.setVariablePosition("", graph_state[WORLD_JOINT_ROLL]);
    robot_state.setVariablePosition("torso_joint1", graph_state[TORSO_JOINT1]);
    robot_state.setVariablePosition("limb_right_joint1", graph_state[LIMB_JOINT1]);
    robot_state.setVariablePosition("limb_right_joint2", graph_state[LIMB_JOINT2]);
    robot_state.setVariablePosition("limb_right_joint3", graph_state[LIMB_JOINT3]);
    robot_state.setVariablePosition("limb_right_joint4", graph_state[LIMB_JOINT4]);
    robot_state.setVariablePosition("limb_right_joint5", graph_state[LIMB_JOINT5]);
    robot_state.setVariablePosition("limb_right_joint6", graph_state[LIMB_JOINT6]);
    robot_state.setVariablePosition("limb_right_joint7", graph_state[LIMB_JOINT7]);
    // robot_state.setVariablePosition("", graph_state[HINGE]);
#else
    for (size_t i = 0; i < model->parent_model->getPlanningJoints().size(); ++i) {
        auto varname = model->parent_model->getPlanningJoints()[i];
        robot_state.setVariablePosition(varname, graph_state[i]);
    }
#endif
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

#if USE_UNIQUE_GOAL
    auto goal_z_index = (int)goal->angles.back();
    if (z_index == goal_z_index) {
        ROS_INFO("Found a goal state (%d vs %d)", z_index, goal_z_index);
        return true;
    }
#else
    auto z_val = graph->m_demo_z_values[z_index];
    auto z_goal = goal->angles[0];
    auto goal_thresh = goal->angle_tolerances[0];
    if (fabs(z_goal - z_val) <= goal_thresh) {
        return true;
    }
#endif

    return false;
}

// Return the index of the waypoint on the demonstration whose z value is
// closest to the given z.
int GetClosestZIndex(RomanObjectManipLattice* graph, double z)
{
    auto closest_z_index = -1;
    auto closest_z_dist = std::numeric_limits<double>::infinity();
    for (auto i = 0; i < graph->m_demo_z_values.size(); ++i) {
        auto z_value = graph->m_demo_z_values[i];
        auto dist = std::fabs(z - z_value);
        if (dist < closest_z_dist) {
            closest_z_dist = dist;
            closest_z_index = i;
        }
    }
    return closest_z_index;
}

enum MotionType
{
    MT_NORMAL = 0,
    MT_OBJECT,
    MT_GRASP,
};

// determine the transition type
static
auto reduce_type(
    const smpl::RobotState& from,
    const smpl::RobotState& to,
    TransitionType::Type type)
    -> MotionType
{
    switch (type) {
    // actions that do not move the object
    case TransitionType::OrigStateOrigSucc:
    case TransitionType::OrigStateBridgeSucc:
    case TransitionType::EGraphStateBridgeSucc:
    case TransitionType::PreGraspAmpSucc:
    case TransitionType::SnapSucc:
        return MT_NORMAL;
    // actions that definitely move the object
    case TransitionType::OrigStateZSucc:
    case TransitionType::EGraphStateZSucc:
        return MT_OBJECT;
    // actions that might move the object
    case TransitionType::EGraphStateAdjSucc:
    case TransitionType::ShortcutSucc:
        return MT_OBJECT; // TODO: check motion of object between the two waypoints
    // grasp actions
    case TransitionType::GraspSucc:
    case TransitionType::PreGraspSucc:
        return MT_GRASP;
    }
}

static
auto to_cstring(MotionType type) -> const char*
{
    switch (type) {
    case MT_NORMAL: return "MT_NORMAL";
    case MT_OBJECT: return "MT_OBJECT";
    case MT_GRASP: return "MT_GRASP";
    default: return "<UNKNOWN>";
    }
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
    SetObjectPose(planner->checker, object_pose);
    planner->graph.setObjectPose(object_pose);

    //////////////////////////////////////////////////////////////////////
    // Clear the graph structure. We shouldn't need to do this but it's //
    // preventing a problem somewhere.                                  //
    //////////////////////////////////////////////////////////////////////

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

    auto fake_coord = smpl::WorkspaceCoord{ };
    planner->graph.m_goal_state_id = planner->graph.createState(fake_coord);
    planner->graph.m_goal_entry = planner->graph.getState(planner->graph.m_goal_state_id);

    ////////////////////////////////////////////////////////////////////////////
    // Update the bounds of the object dimension, which is the number of
    // discrete values it can take on. Changing the RobotModel is considered
    // very bad by smpl conventions, but we get lucky in this case. The maximum
    // object index, which changes per demonstration, is used for two things:
    //
    // (1) adjusting the discretization so that maximum and minimum bounds are
    // reachable for bounded state variables. Our discretization of 1 should
    // never need to change here
    //
    // (2) Checking joint limits. This would invalidate/validate some states,
    // but we're clearing the state of the search between each call.
    ////////////////////////////////////////////////////////////////////////////

    planner->model->min_positions[HINGE] = (double)0;
    planner->model->max_positions[HINGE] = (double)(planner->graph.m_demo_z_values.size() - 1);

    //////////////////////////////////////////////////////////////////////////
    // Transform the object-relative demonstration to the world frame using //
    // the object's pose                                                    //
    //////////////////////////////////////////////////////////////////////////

    for (auto& demo : planner->demos) {
        auto transformed_demo = demo;
        for (auto& point : transformed_demo) {
            auto T_obj_robot = smpl::MakeAffine(
                    point[WORLD_JOINT_X],
                    point[WORLD_JOINT_Y],
                    point[WORLD_JOINT_Z],
                    point[WORLD_JOINT_YAW],
                    point[WORLD_JOINT_PITCH],
                    point[WORLD_JOINT_ROLL]);

            // TODO: this is obnoxiously sensitive. Transforming the
            // demonstration to object frame during recording and back to world
            // frame, even if the relative pose is about the same, causes the
            // pose of the robot to be slightly off due to precision. This is
            // causing some of the nominal examples to not work anymore. The
            // rounding here is a hack to avoid that for now, but the planner
            // should be made robust to these situations.
            auto T_world_robot = smpl::Affine3(object_pose * T_obj_robot);
#if 1
            point[WORLD_JOINT_X] = T_world_robot.translation().x();
            point[WORLD_JOINT_Y] = T_world_robot.translation().y();
            point[WORLD_JOINT_Z] = T_world_robot.translation().z();
            smpl::get_euler_zyx(
                    T_world_robot.rotation(),
                    point[WORLD_JOINT_YAW],
                    point[WORLD_JOINT_PITCH],
                    point[WORLD_JOINT_ROLL]);
#else
            point[WORLD_JOINT_X] = round(1000.0 * T_world_robot.translation().x()) / 1000.0;
            point[WORLD_JOINT_Y] = round(1000.0 * T_world_robot.translation().y()) / 1000.0;
            point[WORLD_JOINT_THETA] = round(1000.0 * smpl::get_nearest_planar_rotation(Eigen::Quaterniond(T_world_robot.rotation()))) / 1000.0;
            point[WORLD_JOINT_Z] = round(1000.0 * T_world_robot.translation().z()) / 1000.0;
#endif
        }

        planner->graph.insertExperienceGraphPath(transformed_demo);
    }

    ////////////////////////////
    // Update the start state //
    ////////////////////////////

    auto start_z_index = GetClosestZIndex(&planner->graph, object_start_state);

    auto start = MakePlannerState(
            start_state, planner->model, (double)start_z_index);

    if (!planner->graph.setStart(start)) {
        ROS_ERROR("Failed to set start");
        return false;
    }

    planner->heuristic.updateStart(start);

    ROS_INFO_STREAM("Start state = " << start);

    /////////////////////
    // Update the goal //
    /////////////////////

    auto goal = smpl::GoalConstraint{ };
    goal.type = smpl::GoalType::USER_GOAL_CONSTRAINT_FN;

    // Hijacking the goal state/tolerance vectors to store the goal position of
    // the object. Since we may have multiple demonstrations that modify the
    // object to some degree, and those demonstrations may not hit the exact
    // same z-value, but be close enough, we want to accept more than one
    // unique z value
#if USE_UNIQUE_GOAL
    goal.angles.push_back(GetClosestZIndex(&planner->graph, object_goal_state));
#else
    goal.angles.push_back(object_goal_state);
#endif
    goal.angle_tolerances.push_back(0.05);
    goal.check_goal = IsGoal;

    PlannerAndGoal goal_data;
    goal_data.goal = &goal;
    goal_data.graph = &planner->graph;
    goal.check_goal_user = &goal_data;

    planner->graph.setGoal(goal);

    planner->heuristic.updateGoal(goal);

    //////////////////////////////////////////////////////
    // Find a path from the start state to a goal state //
    //////////////////////////////////////////////////////

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

    /////////////////////////
    // Prepare return path //
    /////////////////////////

    ROS_INFO("Found path through %zu states with cost %d in %d expansions (%f seconds)",
            solution.size(),
            solution_cost,
            planner->search.get_n_expands(),
            planner->search.get_final_eps_planning_time());

    using RobotPath = std::vector<smpl::RobotState>;
    auto path = RobotPath{ };
    if (!planner->graph.extractPath(solution, path)) {
        ROS_ERROR("Failed to extract path");
        return false;
    }

    if (path.size() < 2) {
        // Trivial path?
        return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // Partition the path into several path segments based on the (reduced) //
    // action type                                                          //
    //////////////////////////////////////////////////////////////////////////

    auto segments = std::vector<RobotPath>();
    auto segment_types = std::vector<MotionType>();

    auto segment = RobotPath();

    // Initialize the first segment with the first two waypoint and the type
    // of action between them
    auto& first_waypoint = path[0];
    auto& second_waypoint = path[1];
    auto segment_type = reduce_type(
            first_waypoint,
            second_waypoint,
            (TransitionType::Type)path[1].back());

    segment.push_back(first_waypoint);
    segment.back().pop_back();
    segment.push_back(second_waypoint);
    segment.back().pop_back();

    auto recorded_last = false;
    for (auto i = 2; i < (int)path.size(); ++i) {
        auto& prev_point = path[i - 1];
        auto& point = path[i];

        auto type = reduce_type(prev_point, point, (TransitionType::Type)point.back());

        if (type == segment_type) {
            // This waypoint is part of the segment, add it and remove its type
            // information
            segment.push_back(point);
            segment.back().pop_back();
        } else {
            // This waypoint is not part of the segment, record the previously
            // constructed segment and initialize the current segment with the
            // previous waypoint, this waypoint, and the type of action between
            // them.
            segments.push_back(segment);
            segment_types.push_back(segment_type);
//            if (i == path.size() - 1) {
//                recorded_last = true;
//            }

            segment.clear();
            segment.push_back(prev_point);
            segment.back().pop_back();
            segment.push_back(point);
            segment.back().pop_back();
            segment_type = type;
        }
    }

    if (!recorded_last) {
        segments.push_back(segment);
        segment_types.push_back(segment_type);
    }

    ROS_INFO("Initial segments:");
    for (auto i = 0; i < segments.size(); ++i) {
        ROS_INFO("  %s: %zu waypoints", to_cstring(segment_types[i]), segments[i].size());
    }

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

        ROS_DEBUG("Smooth path of type %s", to_cstring(type));

        if (!smpl::InterpolatePath(*planner->checker, segment)) {
            ROS_ERROR("Failed to interpolate path");
            return false;
        }

        switch (type) {
        case MT_NORMAL:
        {
            ROS_DEBUG("Shortcut path with %zu waypoints", segment.size());
            auto shortcut = RobotPath();
            auto type = smpl::ShortcutType::JOINT_POSITION_VELOCITY_SPACE;
            smpl::ShortcutPath(planner->model, planner->checker, segment, shortcut, type);
            (void)smpl::InterpolatePath(*planner->checker, shortcut);
            segment = std::move(shortcut);
            break;
        }
        case MT_OBJECT:
        case MT_GRASP:
            break;
//        {
//            if (!smpl::InterpolatePath(*planner->checker, segment)) {
//                ROS_ERROR("Failed to interpolate path");
//                return false;
//            }
//            break;
//        }
        }
    }

    ROS_INFO("Smoothed segments:");
    for (auto i = 0; i < segments.size(); ++i) {
        ROS_INFO("  %s: %zu waypoints", to_cstring(segment_types[i]), segments[i].size());
    }

    // Combine sequences of normal and object manipulation trajectories

    {
        auto combined_segments = std::vector<RobotPath>();
        auto combined_segment_types = std::vector<MotionType>();
        auto segment = segments.front();
        auto segment_type = segment_types.front();
        auto finished_last = false;
        for (auto i = 1; i < segments.size(); ++i) {
            auto& seg = segments[i];
            auto type = segment_types[i];

            auto equiv = false;
            switch (segment_type) {
            case MT_NORMAL:
            case MT_OBJECT:
                equiv = (bool)((type == MT_NORMAL) | (type == MT_OBJECT));
                break;
            case MT_GRASP:
                equiv = type == MT_GRASP;
                break;
            }

            if (equiv) {
                // remove the previously-last waypoint to avoid duplicate
                // waypoints, which pisses off the trajectory follower
                segment.pop_back();
                segment.insert(end(segment), begin(seg), end(seg));
            } else {
                // finish this segment
                combined_segments.push_back(segment);
                combined_segment_types.push_back(segment_type);

                // initialize the next segment with this segment
                segment = seg;
                segment_type = type;
            }
        }
        combined_segments.push_back(segment);
        combined_segment_types.push_back(segment_type);

        segments = std::move(combined_segments);
        segment_types = std::move(combined_segment_types);
    }

    ROS_INFO("Combined segments:");
    for (auto i = 0; i < segments.size(); ++i) {
        ROS_INFO("  %s: %zu waypoints", to_cstring(segment_types[i]), segments[i].size());
    }


    //////////////////////////////////////////////////////////////////////
    // Convert to a sequence of interleaved trajectory/gripper commands //
    //////////////////////////////////////////////////////////////////////

    auto MakeRobotTrajectory = [&](const RobotPath& path)
    {
        robot_trajectory::RobotTrajectory traj(
                start_state.getRobotModel(),
                "right_arm_torso_base");
        for (auto& point : path) {
            auto state = moveit::core::RobotState(start_state);
            UpdateRobotState(state, point, planner->model);
            traj.addSuffixWayPoint(state, 1.0);
        }

        trajectory_processing::IterativeParabolicTimeParameterization profiler;
        profiler.computeTimeStamps(traj);

        ROS_DEBUG("durations:");
        for (auto i = 0; i < traj.getWayPointCount(); ++i) {
            ROS_DEBUG("  %3d: %f, %f", i, traj.getWayPointDurations()[i], traj.getWayPointDurationFromStart(i));
        }

        return traj;
    };

    // create robot trajectory for the first segment
//    auto& first_segment = segments.front();
//    commands->push_back(smpl::make_unique<TrajectoryCommand>(MakeRobotTrajectory(first_segment)));


    for (auto i = 0; i < segments.size(); ++i) {
        ROS_DEBUG("Add open gripper command!");
        if (segment_types[i] == MT_GRASP) {
            commands->push_back(smpl::make_unique<GripperCommand>(true));
        }


        ROS_DEBUG("Add trajectory of length %zu", segments[i].size());
        auto cmd = smpl::make_unique<TrajectoryCommand>(MakeRobotTrajectory(segments[i]));
        commands->push_back(std::move(cmd));

        ROS_DEBUG("Add close gripper command!");
        if (segment_types[i] == MT_GRASP) {
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

