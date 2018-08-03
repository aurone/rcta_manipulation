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

#include "object_manip_heuristic.h"
#include "roman_workspace_lattice_action_space.h"

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

struct ObjectManipPlanner
{
    ObjectManipModel*                       model;
    smpl::PlanningParams                    params;
    smpl::WorkspaceLatticeEGraph            graph;
    RomanWorkspaceLatticeActionSpace        actions;
    smpl::ObjectManipulationHeuristic       heuristic;
    smpl::ARAStar                           search;

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
