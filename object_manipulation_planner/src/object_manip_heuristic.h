#ifndef SMPL_OBJECT_MANIP_HEURISTIC_H
#define SMPL_OBJECT_MANIP_HEURISTIC_H

#include <boost/functional/hash.hpp>

#include <smpl/angles.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

#include "roman_workspace_lattice_egraph.h"

namespace smpl {
class ExperienceGraphExtension;
class ExtractRobotStateExtension;
}

struct PhiCoordHash
{
    using argument_type = PhiCoord;
    using result_type = size_t;
    auto operator()(const argument_type& coord) const -> result_type
    {
        auto seed = size_t(0);
        boost::hash_combine(seed, coord[0]);
        boost::hash_combine(seed, coord[1]);
        boost::hash_combine(seed, coord[2]);
        boost::hash_combine(seed, coord[3]);
        return seed;
    }
};

/// * Only works with RomanObjectManipLattice
/// * Suggests destination states for snap actions by comparing phi coordinates
/// * Provides heuristic that estimates cost to come into contact with the
///   object constraint manifold plus the cost to manipulate the object to its
///   goal position.
/// * Only understands the user-defined goal type, specified in
///   ObjectManipPlanner, which stores the goal position for the object within
///   the same members used by joint space goals.
class ObjectManipHeuristic :
    public smpl::RobotHeuristic,
    public smpl::ExperienceGraphHeuristicExtension
{
public:

    smpl::ExperienceGraphExtension* eg = NULL;
    smpl::ExtractRobotStateExtension* extract_state = NULL;
    smpl::PointProjectionExtension* project_to_point = NULL;

    std::vector<int> egraph_goal_heuristics;

    double w_egraph = 5.0;

    double heading_thresh = 0.1;
    double theta_db = smpl::to_radians(2.0);
    double pos_db = 0.1;
    double theta_normalizer = 0.05 / smpl::to_radians(45.0);
    int h_base_weight = 10;

    // whether to combine the contact and base heuristics via sum or max
    enum struct CombinationMethod
    {
        Sum = 0,
        Max = 1
    } combination = CombinationMethod::Max;

    bool use_rotation = false;
    int heading_condition = 0; // 0 = discrete, or 1 = continuous + threshold, 2 = none
    bool disc_rotation_heuristic = true;
    bool disc_position_heuristic = true;

    // map from z value to all phi cells on the demonstration with that same z value
    smpl::hash_map<int, std::vector<PhiCoord>> z_to_phi;
    smpl::hash_map<int, std::vector<PhiCoord>> z_to_pre_phi;

    smpl::hash_map<
        int,
        std::vector<smpl::ExperienceGraph::node_id>>
    z_to_egraph_node;

    // map from all 3d cells on the demonstration to the heuristic distance
    // minimum heuristic path cost to the any cell with that same
    smpl::hash_map<PhiCoord, int, PhiCoordHash> phi_heuristic;

    smpl::hash_map<PhiCoord, int, PhiCoordHash> pre_phi_heuristic;

    /// \name Required ExperienceGraphHeuristicExtension Interface
    ///@{
    void getEquivalentStates(
        int state_id,
        std::vector<int>& ids) override;

    void getShortcutSuccs(
        int state_id,
        std::vector<int>& ids) override;
    ///@}

    /// \name Required RobotHeuristic Interface
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    void updateGoal(const smpl::GoalConstraint& goal) override;

    /// \name Required Heuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

    auto getExtension(size_t class_code) -> Extension* override;
};

bool Init(ObjectManipHeuristic* heur, smpl::RobotPlanningSpace* space);

#endif

