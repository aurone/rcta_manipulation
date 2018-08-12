#ifndef SMPL_OBJECT_MANIP_HEURISTIC_H
#define SMPL_OBJECT_MANIP_HEURISTIC_H

#include <smpl/angles.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

namespace smpl {
class ExperienceGraphExtension;
class ExtractRobotStateExtension;
}

class ObjectManipulationHeuristic :
    public smpl::RobotHeuristic,
    public smpl::ExperienceGraphHeuristicExtension
{
public:

    smpl::ExperienceGraphExtension* eg = NULL;
    smpl::ExtractRobotStateExtension* extract_state = NULL;
    smpl::PointProjectionExtension* project_to_point = NULL;

    std::vector<int> egraph_goal_heuristics;

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

    bool init(smpl::RobotPlanningSpace* space);

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

#endif
