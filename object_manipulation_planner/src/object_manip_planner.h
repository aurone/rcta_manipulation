#ifndef OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_PLANNER_H
#define OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_PLANNER_H

// standard includes
#include <string>

// system includes
#include <Eigen/Dense>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice_egraph.h>
#include <smpl/planning_params.h>
#include <smpl/search/arastar.h>
#include <smpl/search/experience_graph_planner.h>

#include "object_manip_heuristic.h"
#include "roman_workspace_lattice_action_space.h"
#include "workspace_lattice_egraph_roman.h"

namespace smpl {
class CollisionChecker;
class OccupancyGrid;
} // namespace smpl

namespace moveit {
namespace core {
class RobotState;
} // core
} // moveit

namespace robot_trajectory {
class RobotTrajectory;
} // namespace robot_trajectory

class ObjectManipModel;

struct ObjectManipPlannerParams
{
    // when the base distance is above this threshold, the base theta
    // term in the heuristic includes turning toward the nearest e-graph
    // state, and then turning to the orientation of the e-graph state;
    // otherwise, the base theta term only includes turning toward the
    // nearest e-graph state

    // tolerance before the base theta term is dropped to 0
    bool use_rotation = false;

    // determine base rotation distances in (radianized) cells or radians
    bool disc_rotation_heuristic = true;
    // if continuous, tolerance before the base rotation term is dropped to 0
    double rot_db = smpl::to_radians(2);

    enum HeadingCondition{ Discrete = 0, Continuous = 1, Never = 2 } heading_condition;
    // (heading condition = continuous and position distance <= thresh) -> ignore heading
    double heading_thresh = 0.05;

    // determine base position distance in (meterized) cells or meters
    bool disc_position_heuristic = true;

    // if continuous, tolerance before the base position term is dropped to 0
    double pos_db = 0.1;

    // for combining position + rotation terms
    double rot_weight = 0.45 / smpl::to_radians(45);

    double base_weight = 10.0;
    enum CombinationMethod{ Sum = 0, Max = 1 } combination = CombinationMethod::Max;
};

struct ObjectManipPlanner
{
    smpl::CollisionChecker*                 checker;
    ObjectManipModel*                       model;
    RomanWorkspaceLatticeEGraph             graph;
    RomanWorkspaceLatticeActionSpace        actions;
    ObjectManipHeuristic                    heuristic;
    smpl::ExperienceGraphPlanner            search;

    ObjectManipPlanner();
};

bool Init(
    ObjectManipPlanner* planner,
    ObjectManipModel* model,
    smpl::CollisionChecker* checker,
    smpl::OccupancyGrid* grid);

bool LoadDemonstrations(ObjectManipPlanner* planner, const std::string& path);

bool PlanPath(
    ObjectManipPlanner* planner,
    const moveit::core::RobotState& start_state,
    const Eigen::Affine3d& object_pose,
    double object_start_state,
    double object_goal_state,
    double allowed_time,
    robot_trajectory::RobotTrajectory* trajectory);

#endif
