#ifndef SMPL_OBJECT_MANIP_HEURISTIC_H
#define SMPL_OBJECT_MANIP_HEURISTIC_H

#include <boost/functional/hash.hpp>

#include <smpl/angles.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

#include "roman_object_manip_lattice.h"

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
        for (auto i = 0; i < coord.size(); ++i) {
            boost::hash_combine(seed, coord[i]);
        }
        return seed;
    }
};

struct HeuristicCoord
{
    PhiCoord phi;
    int z;
};

auto operator<<(std::ostream& o, const HeuristicCoord& coord) -> std::ostream&;
bool operator==(const HeuristicCoord& a, const HeuristicCoord& b);

struct HeuristicCoordHash
{
    using argument_type = HeuristicCoord;
    using result_type = size_t;
    auto operator()(const argument_type& coord) const -> result_type
    {
        auto seed = size_t(0);
        for (auto i = 0; i < coord.phi.size(); ++i) {
            boost::hash_combine(seed, coord.phi[i]);
        }
        boost::hash_combine(seed, coord.z);
        return seed;
    }
};

// A heuristic object implementing the following heuristic:
//
// When a state is not coincident with the demonstration (the discretized phi
// coordinate of the state does not match any state on the demonstration), the
// heuristic value of the state is an estimate of the cost to move to the
// nearest 'pregrasp' phi coordinate of a state on the demonstration that has
// the same z value, plus an underestimate of the cost to move from the
// 'pregrasp' phi coordinate to its corresponding nominal phi coordinate, plus
// an underestimate of the cost to move through the demonstration to a state on
// the demonstration that has the same z value as the goal state.
//
// When a state is coincident with the demonstration, the heuristic similarly
// includes an underestimate of the cost to move through the demonstration to
// a goal state, but does not include the cost to reach the demonstration or
// to first move through a 'pregrasp' phi.
//
// * Only works with RomanObjectManipLattice
// * Suggests destination states for snap actions by comparing phi coordinates
// * Provides heuristic that estimates cost to come into contact with the
//   object constraint manifold plus the cost to manipulate the object to its
//   goal position.
// * Only understands the user-defined goal type, specified in
//   ObjectManipPlanner, which stores the goal position for the object within
//   the same members used by joint space goals.
class ObjectManipHeuristic :
    public smpl::RobotHeuristic,
    public smpl::ExperienceGraphHeuristicExtension
{
public:

    // Required interfaces
    smpl::ExperienceGraphExtension* eg = NULL;
    smpl::ExtractRobotStateExtension* extract_state = NULL;
    smpl::PointProjectionExtension* project_to_point = NULL;

    // The heuristic values of all states on the demonstration
    std::vector<int> egraph_goal_heuristics;

    // map from z value to all states on the demonstration with that same z
    // value. The full state is used to create more informed heuristics, but
    // less admissible heuristic values.
    smpl::hash_map<int, std::vector<smpl::ExperienceGraph::node_id>> z_to_egraph_node;

    // map from z value to the phi coordinates of all states on the
    // demonstration with that same z value
    smpl::hash_map<int, std::vector<PhiCoord>> z_to_phi;

    // map from z value to the 'pregrasp' phi coordinates of all states on the
    // demonstration with that same z value
    smpl::hash_map<int, std::vector<PhiCoord>> z_to_pre_phi;

    // map from the phi coordinates of all states on the demonstration to the
    // cost of the minimum path that goes through the demonstration to a state
    // on the demonstration with the goal z value
    smpl::hash_map<HeuristicCoord, int, HeuristicCoordHash> phi_heuristic;

    // map from the 'pregrasp' phi coordinates of all states on the
    // demonstration to the cost of the minimum path that goes through the
    // demonstration to a state on the demonstration with the goal z value
    smpl::hash_map<HeuristicCoord, int, HeuristicCoordHash> pre_phi_heuristic;

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

    enum HeadingCondition {
        Discrete = 0,
        Continuous,
        None,
    };

    bool use_rotation = false;
    int heading_condition = HeadingCondition::Discrete;
    bool disc_rotation_heuristic = true;
    bool disc_position_heuristic = true;

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

