#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_planners_sbpl/planner/moveit_robot_model.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/robot_model.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/search/arastar.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h> // NOTE: actually smpl_ros
#include <smpl/graph/workspace_lattice_egraph.h>

//#include "workspace_lattice_egraph.h"

namespace smpl = sbpl::motion;

// A RobotModel struct that takes an existing RobotModel and extends it by
// adding a single degree-of-freedom.
struct ObjectManipulationModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface,
    public virtual smpl::InverseKinematicsInterface,
    public virtual smpl::RedundantManipulatorInterface
{
    smpl::RobotModel* robot_model = NULL;

    // cache the underlying interfaces that we wrap
    smpl::ForwardKinematicsInterface* fk_iface = NULL;
    smpl::InverseKinematicsInterface* ik_iface = NULL;
    smpl::RedundantManipulatorInterface* rm_iface = NULL;

    double min_object_pos = 0.0;
    double max_object_pos = 0.0;

//    ObjectManipulationModel() = default;

    double minPosLimit(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->minPosLimit(vidx);
        } else {
            return this->min_object_pos;
        }
    }

    double maxPosLimit(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->maxPosLimit(vidx);
        } else {
            return this->max_object_pos;
        }
    }

    bool hasPosLimit(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->hasPosLimit(vidx);
        } else {
            return true;
        }
    }

    bool isContinuous(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->isContinuous(vidx);
        } else {
            return false;
        }
    }

    double velLimit(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->velLimit(vidx);
        } else {
            return 0.0;
        }
    }

    double accLimit(int vidx) const override
    {
        if (vidx < robot_model->jointCount()) {
            return robot_model->accLimit(vidx);
        } else {
            return 0.0;
        }
    }

    bool checkJointLimits(const smpl::RobotState& state, bool verbose = false) override
    {
        auto ovar = state.back();
        return robot_model->checkJointLimits(state) &&
                ovar >= this->min_object_pos &&
                ovar <= this->max_object_pos;
    }

    auto getExtension(size_t class_code) -> smpl::Extension* override
    {
        if (class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) {
            return this->fk_iface ? this : NULL;
        } else if (class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>()) {
            return this->ik_iface ? this : NULL;
        } else if (class_code == smpl::GetClassCode<smpl::RedundantManipulatorInterface>()) {
            return this->rm_iface ? this : NULL;
        } else if (class_code == smpl::GetClassCode<smpl::RobotModel>()) {
            return this;
        }

        return NULL;
    }

    // we should just be able to use the underlying robot model's forward
    // kinematics
    auto computeFK(const smpl::RobotState& state) -> Eigen::Affine3d override
    {
        return this->fk_iface->computeFK(state);
    }

    // same for ik...
    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override
    {
        if (!this->ik_iface->computeIK(pose, start, solution, option)) {
            return false;
        }

        // push back the object variable
        solution.push_back(start.back());
        return true;
    }

    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        std::vector<smpl::RobotState>& solutions,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override
    {
        if (!this->ik_iface->computeIK(pose, start, solutions, option)) {
            return false;
        }
        for (auto& sol : solutions) {
            sol.push_back(start.back());
        }
        return true;
    }

    const int redundantVariableCount() const override
    {
        return this->rm_iface->redundantVariableCount() + 1;
    }

    const int redundantVariableIndex(int rvidx) const override
    {
        if (rvidx < this->rm_iface->redundantVariableCount()) {
            return this->rm_iface->redundantVariableIndex(rvidx);
        } else {
            // the object variable is always the last variable in the robot state
            return robot_model->jointCount();
        }
    }

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& state,
        smpl::RobotState& solution)
    {
        return false;
    }

#if 0


    // the redundant manipulator interface will be a little different...
    // we need to tell smpl that the object dimension is a 'free angle'
#endif
};

//ObjectManipulationModel::ObjectManipulationModel()
//{
//    // given a state space representation of the robot
//    // (base/x, base/y, base/theta, torso, arm/1, arm/2, arm/3, arm/4, arm/5, arm/6, arm/7)
//    // transform a given state to workspace representation (
//    std::vector<std::string> var_names =
//    {
//        "base/x",
//        "base/y",
//        "base/theta",
//        "torso",
//        "arm/free",
//        "ee/x",
//        "ee/y",
//        "ee/z",
//        "ee/yaw",
//        "object",
//    };
//    setPlanningJoints(var_names);
//}

// Some design decisions here...
// * Do we want to construct a RobotModel explicitly defining the workspace
// representation and pass that to ManipLattice, or to construct a standard
// RobotModel and pass that to something like Workspace Lattice
// * Do we want to include the object dimension as part of the RobotModel? We'll
// (probably) need to if we want to use an existing graph structure
// 4-ish combinations
//
// * ManipLattice     w/  object variable
//
// bounds need to be given for the variable to affect discretization. Do we
// discretize values from Z or do we just consider all the discrete values
// from the demonstrated trajectory.
//
// * ManipLattice     w/o object variable
//
// * WorkspaceLattice w/  object variable
//
// * WorkspaceLattice w/o object variable
//
// Construct RobotModel as you usually would, describing the variables of your
// robot that the planner may reason about. WorkspaceLattice requires a
// ForwardKinematicsInterface, InverseKinematicsInterface, and a
// RedundantManipulatorInterface.
//
// * no/no   -
// * no/yes  -
// * yes/no  -
// * yes/yes -
//
// Z = all possible values of z from the demonstration

template <class T>
T* GetExtension(smpl::Extension* extension)
{
    auto* e = extension->getExtension(smpl::GetClassCode<T>());
    return dynamic_cast<T*>(e);
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
    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);

    auto group_name = "right_arm_and_torso";

    // We basically have to be a ROS node for this to work. We could use
    // RobotModelLoader to load the RobotModel from urdf/srdf strings, but
    // we still wouldn't be able to load kinematics solvers without doing it
    // manually...
    ros::init(argc, argv, "object_manip_planner");
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader loader;
    auto robot_model = loader.getModel();
    if (!robot_model) {
        ROS_ERROR("Failed to load Robot Model");
        return 1;
    }

    // want to include the object degree-of-freedom as a free variable in the
    // robot model to be able to use workspace lattice directly
    sbpl_interface::MoveItRobotModel planning_model;
    if (!planning_model.init(robot_model, group_name)) {
        ROS_ERROR("Failed to initialize robot model");
        return 1;
    }

    ObjectManipulationModel omanip;
    omanip.robot_model = &planning_model;
    omanip.fk_iface = GetExtension<smpl::ForwardKinematicsInterface>(&planning_model);
    omanip.ik_iface = GetExtension<smpl::InverseKinematicsInterface>(&planning_model);
    omanip.min_object_pos = 0.0;
    omanip.max_object_pos = 1.0;

    sbpl::collision::CollisionSpace cspace;
    double size_x, size_y, size_z;
    double resolution;
    double origin_x, origin_y, origin_z;
    double max_dist;
    auto grid = sbpl::OccupancyGrid(
            size_x, size_y, size_z,
            resolution,
            origin_x, origin_y, origin_z,
            max_dist);

    sbpl::collision::CollisionModelConfig config;
    if (!sbpl::collision::CollisionModelConfig::Load(nh, config)) {
        ROS_ERROR("Failed to load Collision Model Configuration");
        return 1;
    }

    std::vector<std::string> planning_variables;
    if (!cspace.init(&grid, *robot_model->getURDF().get(), config, group_name, planning_variables)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    // TODO: set up the scene (set the initial robot state)
    SV_SHOW_INFO(cspace.getCollisionRobotVisualization());

    smpl::WorkspaceLatticeEGraph graph;

    smpl::WorkspaceLattice::Params p;
    smpl::PlanningParams params;
    if (!graph.init(&omanip, &cspace, &params, p)) {
        ROS_ERROR("Failed to initialize Workspace Lattice E-Graph");
        return false;
    }

    // TODO: heuristic is only built from the grid during
    // construction...that's pretty fucking stupid
    smpl::DijkstraEgraphHeuristic3D heuristic;
    if (!heuristic.init(&graph, &grid)) {
        ROS_ERROR("Failed to initialize Dijkstra E-Graph Heuristic 3D");
        return false;
    }

    sbpl::ARAStar search(&graph, &heuristic);
    search.allowPartialSolutions(false);
    search.setTargetEpsilon(1.0);
    search.setDeltaEpsilon(1.0);
    search.setImproveSolution(true);
    search.setBoundExpansions(true);

    // level out the end effector

    // goal is to open the object all the way
    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;

    sbpl::ARAStar::TimeParameters timing;
    timing.bounded = true;
    timing.improve = true;
    timing.max_allowed_time_init = std::chrono::seconds(10);
    timing.max_allowed_time = std::chrono::seconds(10);
    std::vector<int> solution;
    int solution_cost;
    bool res = search.replan(timing, &solution, &solution_cost);
    if (!res) {
        ROS_ERROR("Failed to plan path");
        return 1;
    }

    return 0;
}

